#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp>

double RobotV_ = 0;
double YawRate_ = 0;

double x_;
double y_;
double th_;

double vx_;
double vy_;
double vth_;

union odometry_get
{
    float odometry_float;
    unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;


ros::Time current_time_, last_time_;
//tf::TransformBroadcaster odom_broadcaster_;

const unsigned char header[2] = {0x55, 0xaa};
//const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char ender[2] = {0x66, 0xcc};

const double ROBOT_RADIUS = 90.00;//转换单位为mm
const double ROBOT_LENGTH = 180.00;//转换单位为mm

union serial_send
{
    int d;
    unsigned char data[4];
}leftdata,rightdata;

union receiveHeader
{
	int d;
	unsigned char data[2];
} receive_header;

union odometry
{
	int odoemtry_int;
	unsigned char odometry_char[4];
}vel_left, vel_right;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_ = msg.linear.x * 1000;//转换单位为mm/s
	YawRate_ = msg.angular.z;     
}

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

//创建一个serial类
serial::Serial sp;
void serial_initial()
{
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(9600);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        //return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        //return -1;
    } 
}

void writeSpeed(double RobotV, double YawRate)
{
	unsigned char buf[16] = {0};
	int i, length = 0;
	double r = RobotV / YawRate;

    // 计算左右轮期望速度
	if(RobotV == 0)
	{
		leftdata.d = -YawRate * ROBOT_RADIUS;
		rightdata.d = YawRate * ROBOT_RADIUS;
	} 
    else if(YawRate == 0)
	{
		leftdata.d = RobotV;
		rightdata.d = RobotV;
	}
	else
	{
		leftdata.d  = YawRate * (r - ROBOT_RADIUS);
		rightdata.d = YawRate * (r + ROBOT_RADIUS);
	}

    // 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];

    // 设置机器人左右轮速度
	length = 8;  
    buf[6] = length;

	for(i = 0; i < 4; i++)
	{
		buf[i + 2] = rightdata.data[i];
		buf[i + 7] = leftdata.data[i];
	}

    //消息尾 
	buf[2 + length + 1] = ender[0];
	buf[2 + length + 2] = ender[1];

    //ROS_INFO("leftdata: %d", leftdata.d);
    //ROS_INFO("rightdata: %d \n", rightdata.d);  

    for(i=0;i<13;i++)
    {
        //ROS_INFO("buf %x", buf[i]);
    }
    
    sp.write(buf, 13);
}

void read_speed()
{
    int i,t;
    unsigned char buf[200];

    // 读取串口缓冲区字节数
    size_t n = sp.available();
    ros::Time curr_time;

    if(n!=0)
    {
        sp.read(buf, n);

        for(i=0;i<n;i++)
        {
            if(buf[i]==header[0] && buf[i+1]==header[1]
            && buf[i+22]==ender[0] && buf[i+23]==ender[1])
            {
                //x_data,y_data,theta_data,vel_linear,vel_angular;
                for(t=0; t < 4; t++)
                {
                    x_data.odometry_char[t] = buf[i + t + 2];
                    y_data.odometry_char[t]  = buf[i + t + 6];
                    vel_linear.odometry_char[t] = buf[i + t + 10];
                    vel_angular.odometry_char[t] = buf[i + t + 14];
                    theta_data.odometry_char[t] = buf[i + t + 18];
                }
            }    
            break;
        }
        // 积分计算里程计信息
        //vx_  = ((double)vel_right.odoemtry_int + (double)vel_left.odoemtry_int) / 2 / 1000;
        //vth_ = ((double)vel_right.odoemtry_int - (double)vel_left.odoemtry_int) / ROBOT_LENGTH;
        vx_ = (double)vel_linear.odometry_float/1000;
        vth_ = (double)vel_angular.odometry_float;

        curr_time = ros::Time::now();

        double dt = (curr_time - last_time_).toSec();
        //double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
        //double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
        //double delta_th = vth_ * dt;

       // x_ += delta_x;
       // y_ += delta_y;
       // th_ += delta_th;
       x_ = (double)x_data.odometry_float/1000;
       y_ = (double)y_data.odometry_float/1000;
       th_ = (double)theta_data.odometry_float;

        last_time_ = curr_time;

        ROS_INFO("n: %ld", n);
        
        ROS_INFO("vx_: %f", vx_);
        ROS_INFO("vth_: %f \n", vth_);  

        //ROS_INFO("vx_: %f", vx_);
        //ROS_INFO("vth_: %f \n", vth_); 

        ROS_INFO("x_: %f", x_);
       //tmp ROS_INFO("y_: %f", y_);        
       // ROS_INFO("th_: %f \n", th_);        

        //ROS_INFO("curr_time: ros::Time \n", curr_time);         

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"uart");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster_;

    ros::Subscriber sub = n.subscribe("cmd_vel", 50, cmdCallback);
    
    serial_initial();

    ros::Time::init();
	current_time_ = ros::Time::now();
	last_time_ = ros::Time::now();
	
    //定义发布消息的名称
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50);    

    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("enter while(1)");
    unsigned char buf[13]={0,1,2,3,4,5,6,7,8,9,10,11,12};
    while(ros::ok())//3.28 先验证工控机串口发一次，再从底层板接收发送的数据
    {      
		ros::spinOnce(); 
        //writeSpeed(1,0);
        writeSpeed(RobotV_,YawRate_);
        //sp.write(buf, 13);
        read_speed(); 

        current_time_ = ros::Time::now();
        // 发布TF
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_footprint";

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(th_);
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster_.sendTransform(odom_trans);

        // 发布里程计消息
        nav_msgs::Odometry msgl;
        msgl.header.stamp = current_time_;
        msgl.header.frame_id = "odom";

        msgl.pose.pose.position.x = x_;
        msgl.pose.pose.position.y = y_;
        msgl.pose.pose.position.z = 0.0;
        msgl.pose.pose.orientation = odom_quat;
        msgl.pose.covariance = odom_pose_covariance;

        msgl.child_frame_id = "base_footprint";
        msgl.twist.twist.linear.x = vx_;
        msgl.twist.twist.linear.y = vy_;
        msgl.twist.twist.angular.z = vth_;
        msgl.twist.covariance = odom_twist_covariance;
    
        pub.publish(msgl);

        //获取缓冲区内的字节数
        /*
        size_t n = sp.available();
        if(n!=0)
        {
            ROS_INFO_STREAM("read data");
            
            uint8_t buffer[1024];
            uint16_t trans_buff[1024];
            uint16_t data_serial;
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {
                if(buffer[i]<=70 && buffer[i]>=65)
                    trans_buff[i]=(buffer[i]-48-7);
                else
                    trans_buff[i]=(buffer[i]-48);                    
                
                trans_buff[i]=trans_buff[i]<<((n-1-i)*4);
                data_serial+=trans_buff[i];
                //10进制的方式打印到屏幕
                //std::cout << (buffer[i] & 0xff) << " ";
                //std::cout << (trans_buff[i] & 0xff) << " ";
            }
            std::cout << n << " ";            
            std::cout << (data_serial) << " ";
            data_serial=0;

            std::cout << std::endl;
            //把数据发送回去
            //sp.write(buffer, n);
            
        }*/
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;

}
#include <ros/ros.h>
#include <signal.h>
#include <ros/package.h>
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "raspimouse_with_jetson_nano/MotorFreqs.h"
#include "raspimouse_with_jetson_nano/TimedMotion.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include "sensor_msgs/Imu.h"

using namespace ros;

// bool setPower(bool);
void setFreqs(int left, int right);

// void onSigint(int);
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackTimedMotion(raspimouse_with_jetson_nano::TimedMotion::Request&, raspimouse_with_jetson_nano::TimedMotion::Response&);
void callbackRaw(const raspimouse_with_jetson_nano::MotorFreqs::ConstPtr& msg);
void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg);
void callback9Axis(const sensor_msgs::Imu::ConstPtr& msg);

bool is_on = false;
bool in_cmdvel = false;
Time last_cmdvel;
Time cur_time;
Time send_time;

geometry_msgs::Twist vel;
double odom_x,odom_y,odom_theta;

bool imu_flag = false;

// bool setPower(bool on)
// {

// 	std::ofstream motor_dir("/sys/class/gpio/gpio149/direction");
// 	if(not motor_dir.is_open()){
// 		std::cout << "motor_dir_fail_open" << std::endl;
// 		return false;
// 	}

// 	motor_dir << "out" << std::endl;

// 	std::ofstream motor_value("/sys/class/gpio/gpio149/value");
// 	if(not motor_value.is_open()){
// 		std::cout << "motor_val_fail_open" << std::endl;
// 		return false;
// 	}

// 	motor_value << (on ? '1' : '0') << std::endl;
// 	is_on = on;
// 	return true;
// }

void setFreqs(int left, int right)
{
	int left_period = 0;
	if(left != 0)
		left_period = 1000000000 / std::abs(left);
	else
		left_period = 0;
	int right_period =0;
	if(right != 0)
		right_period = 1000000000 / std::abs(right);
	else
		right_period = 0;

	std::ofstream motor_right_pwm_period("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm2/period");
	motor_right_pwm_period << right_period << std::endl;
	std::ofstream motor_left_pwm_period("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm0/period");
	motor_left_pwm_period << left_period << std::endl;
	std::ofstream motor_right_pwm_duty("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm2/duty_cycle");
	motor_right_pwm_duty << right_period /2 << std::endl;
	std::ofstream motor_left_pwm_duty("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm0/duty_cycle");
	motor_left_pwm_duty << left_period /2 << std::endl;
	std::ofstream motor_right_pwm_en("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm2/enable");
	motor_right_pwm_en << 1 << std::endl;
	std::ofstream motor_left_pwm_en("/sys/devices/7000a000.pwm/pwm/pwmchip0/pwm0/enable");
	motor_left_pwm_en << 1 << std::endl;
	std::ofstream motor_right_dir_pin("/sys/class/gpio/gpio76/direction");
	motor_right_dir_pin << "out" << std::endl;
	std::ofstream motor_left_dir_pin("/sys/class/gpio/gpio200/direction");
	motor_left_dir_pin << "out" << std::endl;
	std::ofstream motor_right_value_pin("/sys/class/gpio/gpio76/value");
	motor_right_value_pin << 1 << std::endl;
	std::ofstream motor_left_value_pin("/sys/class/gpio/gpio200/value");
	if(left <0)
		motor_left_value_pin << 1 << std::endl;
	else
		motor_left_value_pin << 0 << std::endl;
	if(right <0)
		motor_right_value_pin << 0 << std::endl;
	else
		motor_right_value_pin << 1 << std::endl;
}

// void onSigint(int sig)
// {
// 	// setPower(false);
// 	exit(0);
// }

bool callbackOn(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	// if(not setPower(true))
	// 	return false;

	response.message = "ON";
	response.success = true;
	return true;
}

bool callbackOff(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
	// if(not setPower(false))
	// 	return false;

	response.message = "OFF";
	response.success = true;
	return true;
}

bool callbackTimedMotion(raspimouse_with_jetson_nano::TimedMotion::Request& request, raspimouse_with_jetson_nano::TimedMotion::Response& response)
{
	if(not is_on){
		ROS_INFO("Motors are not enpowered");
		return false;
	}

	std::ofstream ofs("/dev/rtmotor0");
	if(not ofs.is_open()){
		ROS_ERROR("Cannot open /dev/rtmotor0");
		return false;
	}

	ofs << request.left_hz << ' ' << request.right_hz << ' '
		<< request.duration_ms << std::endl;

	response.success = true;
	return true;
}

void callbackRaw(const raspimouse_with_jetson_nano::MotorFreqs::ConstPtr& msg)
{
	setFreqs(msg->left_hz, msg->right_hz);
}

void callbackCmdvel(const geometry_msgs::Twist::ConstPtr& msg)
{

	vel.linear.x = msg->linear.x;

	if(!imu_flag)
		vel.angular.z = msg->angular.z;

	double forward_hz = 80000.0*msg->linear.x/(9*3.141592);
	double rot_hz = 400.0*msg->angular.z/3.141592;
	setFreqs((int)round(forward_hz-rot_hz), (int)round(forward_hz+rot_hz));
	in_cmdvel = true;
	last_cmdvel = Time::now();
}

void callback9Axis(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(not imu_flag){
		ROS_INFO("9-axis sensor mode");
		imu_flag = true;
	}

	vel.angular.z = round(msg->angular_velocity.z * 20)/20; // cut less than 0.05[rad/s]
}

nav_msgs::Odometry send_odom(void)
{
	cur_time = Time::now();

	double dt = cur_time.toSec() - send_time.toSec();
	odom_x += vel.linear.x * cos(odom_theta) * dt;
	odom_y += vel.linear.x * sin(odom_theta) * dt;

	odom_theta += vel.angular.z * dt;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_theta);

	static tf2_ros::TransformBroadcaster br;

	geometry_msgs::TransformStamped odom_trans;
 	odom_trans.header.stamp = cur_time;
  	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

  	odom_trans.transform.translation.x = odom_x;
  	odom_trans.transform.translation.y = odom_y;
  	odom_trans.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0, 0, odom_theta);
	odom_trans.transform.rotation.x = q.x();
 	odom_trans.transform.rotation.y = q.y();
	odom_trans.transform.rotation.z = q.z();
	odom_trans.transform.rotation.w = q.w();

	br.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = cur_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	odom.pose.pose.position.x = odom_x;
	odom.pose.pose.position.y = odom_y;
	odom.pose.pose.position.z = 0.0;
  	odom_trans.transform.rotation = odom_quat;
	odom.pose.pose.orientation = odom_quat;

	odom.twist.twist.linear.x = vel.linear.x;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vel.angular.z;

	send_time = cur_time;
	return odom;
}

int main(int argc, char **argv)
{
	setFreqs(0,0);

	init(argc,argv,"motors");
	NodeHandle n;

	std::string onoff;
	if(argc > 1)
		onoff = argv[1];

	// setPower(onoff == "on");
	// signal(SIGINT, onSigint);

	std::ofstream gpio_exp("/sys/class/gpio/export");
	gpio_exp << 149 << std::endl;
	gpio_exp << 200 << std::endl;
	gpio_exp << 76 << std::endl;
	std::ofstream motor_pwm_exp("/sys/devices/7000a000.pwm/pwm/pwmchip0/export");
	motor_pwm_exp << 0 << std::endl;
	motor_pwm_exp << 2 << std::endl;

	last_cmdvel = Time::now();
	cur_time = Time::now();
	send_time = Time::now();

	ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
	ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);
	ServiceServer srv_tm = n.advertiseService("timed_motion", callbackTimedMotion);

	Subscriber sub_raw = n.subscribe("motor_raw", 10, callbackRaw);
	Subscriber sub_cmdvel = n.subscribe("cmd_vel", 10, callbackCmdvel);
	Subscriber sub_9axis = n.subscribe("/imu/data_raw", 10, callback9Axis);

	Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 10);
	odom_x = 0.0;
	odom_y = 0.0;
	odom_theta = 0.0;

	send_time = Time::now();

	Rate loop_rate(10);
	while(ok()){
		if(in_cmdvel and Time::now().toSec() - last_cmdvel.toSec() >= 1.0){
			setFreqs(0,0);
			in_cmdvel = false;
		}

		pub_odom.publish(send_odom());
		spinOnce();
		loop_rate.sleep();
	}

	exit(0);
}

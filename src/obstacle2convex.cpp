#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"


//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>



class Imu2TF
{
	ros::NodeHandle n;
	tf::StampedTransform transform;
	ros::Subscriber IMU_sub_;
	static tf::TransformBroadcaster br_;

	std::string imu_topic_;




	public:
		Imu2TF(std::string& imu_topic) : imu_topic_(imu_topic)
			{

			ROS_INFO("Initializing class");
			IMU_sub_ = n.subscribe(imu_topic_, 10, &Imu2TF::imuCallback, this);
			ROS_INFO("After subscribe");
		}
		
	~Imu2TF()
		{
		}
		

	void imuCallback(const sensor_msgs::Imu& imu_in)
		{
			static tf::TransformBroadcaster br, br2;
			transform.setOrigin( tf::Vector3(0, 0, 0) );
			tf::Quaternion q;

			q=tf::Quaternion(imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z, imu_in.orientation.w);

//			transform.setRotation(tf::Quaternion(imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z, imu_in.orientation.w));

			transform.setRotation(q);
			
//			std::cerr << "  qx:  " << imu_in.orientation.x << "  qy:  " << imu_in.orientation.y << "  qz:  " << imu_in.orientation.z << "  qw:  " << imu_in.orientation.w <<std::endl;
			
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));

			q.setRPY(0,0,-M_PI/2);
			transform.setRotation(q);

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu", "camera_link"));
//			ROS_INFO("I'm in");
		}	
		


};










int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu2tf");
  std::string imu_topic = "imu/data";
    
  Imu2TF converter(imu_topic);

  ros::spin();

  return 0;
}




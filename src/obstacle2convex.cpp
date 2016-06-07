//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"



//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>





class obstacle2convex
{
	ros::NodeHandle n;
	tf::StampedTransform transform;
	ros::Subscriber IMU_sub_;
	static tf::TransformBroadcaster br_;

	std::string imu_topic_;
	ros::Subscriber map_sub_;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		




	public:
		obstacle2convex(std::string& imu_topic) : imu_topic_(imu_topic), it_(n)
			{

			ROS_INFO("Initializing class");
//			IMU_sub_ = n.subscribe(imu_topic_, 10, &obstacle2convex::imuCallback, this);
			map_sub_ = n.subscribe("map", 1, &obstacle2convex::mapCallback, this);
			
			image_pub_ = it_.advertise("/image_frontier", 1);
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
			
			
			ROS_INFO("After subscribe");
		}
		
	~obstacle2convex()
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
		

//////////////////////////////////		
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
		{
			
			cv::Mat grad;
			
			clock_t begin = clock();
			
			clock_t begin_process, end_process;
			double elapsed_secs_process;// = double(end_process - begin_process) / CLOCKS_PER_SEC;
//			end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in process "<< elapsed_secs_process*1000 << " ms"<<std::endl;

			{
			std::cout <<"Map_Info_.resolution  " << map->info.resolution << std::endl;


			ROS_INFO("Received a %d X %d map @ %.3f m/pix",
				map->info.width,
				map->info.height,
				map->info.resolution);
			 } 
			 
			 
			cv_ptr->header = map->header;
		}



};










int main(int argc, char **argv)
{

  ros::init(argc, argv, "obstacle2convex");
  std::string imu_topic = "imu/data";
    
  obstacle2convex converter(imu_topic);

  ros::spin();

  return 0;
}




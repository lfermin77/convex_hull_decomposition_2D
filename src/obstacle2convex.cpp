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


using namespace std;




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

			ROS_INFO("Waiting for map");
			map_sub_ = n.subscribe("map", 1, &obstacle2convex::mapCallback, this);
			
			image_pub_ = it_.advertise("/image_frontier", 1);
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
			
		}
		
	~obstacle2convex()
		{
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

	///////////////////////////////////
	// Occupancy Grid to Image
			begin_process = clock();
			
			cv::Mat Occ_image(map->info.height, map->info.width, CV_8U);
			Occ_image.data = (unsigned char *)(&(map->data[0]) );

			image_pub_.publish(cv_ptr->toImageMsg());
			
			end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in occupancy to image "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

	///////////////////////////////////
	///// Find occupied points
			begin_process = clock();
			
			cv::Mat Occupied = Occ_image>90 & Occ_image<=100;

			vector<cv::Point> Occupied_Points;   // output, locations of non-zero pixels
			cv::findNonZero(Occupied, Occupied_Points);

/*
			float points_in_image=( map->info.height) * (map->info.width);
			float point_fraction = Occupied_Points.size()/( map->info.height * map->info.width);						
			cout << "Points in vector "<<Occupied_Points.size()<< " out of "<< points_in_image << endl;
			cout << "Points in vector "<< point_fraction*100 << "%"<<endl;
*/


			end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in image to vector "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

	//////////////////////////////////
	///// Hull Decomposition
	
	


	//////////////////////
	///// PUBLISH
			grad = Occupied;
			
			
			cv_ptr->encoding = "32FC1";
			grad.convertTo(grad, CV_32F);
			grad.copyTo(cv_ptr->image);////most important

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




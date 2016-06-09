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
	
	ros::Timer timer;

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
			timer = n.createTimer(ros::Duration(0.5), &obstacle2convex::metronomeCallback, this);
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
			
		}
		
	~obstacle2convex()
		{
		}
		
////////////////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
			image_pub_.publish(cv_ptr->toImageMsg());
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
			cv::Mat cleared_area = Occ_image>=0 & Occ_image<=10;

			vector<cv::Point> Occupied_Points, cleared_points;   // output, locations of non-zero pixels
//			cv::findNonZero(Occupied, Occupied_Points);
//			cv::findNonZero(cleared_area,cleared_points);


			float points_in_image=( map->info.height) * (map->info.width);
/*
			float point_fraction = Occupied_Points.size()/( map->info.height * map->info.width);						
			cout << "Points in vector "<<Occupied_Points.size()<< " out of "<< points_in_image << endl;
			cout << "Points in vector "<< point_fraction*100 << "%"<<endl;
*/


			end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in image to vector "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

	//////////////////////////////////
	///// Hull Decomposition
			begin_process = clock();

			vector<cv::Point> convex_hull;			
//			cv::convexHull( Occupied_Points, convex_hull );
//			cv::convexHull( cleared_points, convex_hull );


			cv::Mat dist;
			cv::distanceTransform(~Occupied, dist, CV_DIST_L2, 3);

			std::vector<std::vector<cv::Point> > first_contour;
			cv::findContours(cleared_area, first_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );



			
			cv::Mat will_be_destroyed = dist<2;//.clone();	
//			cv::convexHull( will_be_destroyed, convex_hull );
			dist = dist <2;
			
//			cout <<"big_contours_vector.size() "  << endl;
			
			std::vector<std::vector<cv::Point> > Differential_contour;
			cv::findContours(will_be_destroyed, Differential_contour, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
//			cout <<"Differential_contour.size() " << Differential_contour.size() << endl;
			
			vector<vector<cv::Point> > big_contours_vector;
			for(int i=0; i < Differential_contour.size(); i++){
				float current_area = cv::contourArea(Differential_contour[i]);
//				if(current_area > points_in_image/1000){
				if(current_area > 100){
					big_contours_vector.push_back(Differential_contour[i]);
				}
			}
			
			cv::Mat contour_drawing = cv::Mat::zeros(Occupied.size().height, Occupied.size().width, CV_8UC1);

//			for(int i=0;i < big_contours_vector.size() ; i++) drawContours(contour_drawing, big_contours_vector, i, 255, -1, 8);

			
//			cout <<"big_contours_vector.size() " << big_contours_vector.size() << endl;
			
			
			vector<vector<cv::Point> > hull_vector( big_contours_vector.size() +1);
//			hull_vector[0] = convex_hull;
//			hull_vector[0] = first_contour[0];


			for( int i = 0; i < big_contours_vector.size(); i++ ){  
			   cv::convexHull( big_contours_vector[i], hull_vector[i+1] ); 
			}

//			drawContours(contour_drawing, hull_vector, -1, 128, 3, 8);
			for(int i=0;i < hull_vector.size() ; i++) drawContours(contour_drawing, hull_vector, i, 128, 3, 8);
			for(int i=0;i < big_contours_vector.size() ; i++) drawContours(contour_drawing, big_contours_vector, i, 255, -1, 8);
			for(int i=0;i < first_contour.size() ; i++) drawContours(contour_drawing, first_contour, i, 200, 3, 8);



			end_process=clock();   elapsed_secs_process = double(end_process - begin_process) / CLOCKS_PER_SEC;			std::cerr<<"Time elapsed in convex hull "<< elapsed_secs_process*1000 << " ms"<<std::endl<<std::endl;

	//////////////////////
	///// PUBLISH
//			grad = Occupied;
//			grad = dist;
			grad = contour_drawing;
			
			cv::flip(grad,grad,0);
			
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




#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"

using namespace std;

class PointFilter {
	public:
		ros::NodeHandle nh;
		ros::Publisher filtered_pub;
		ros::Subscriber cloud_sub;
		PointFilter(ros::NodeHandle input_handler);
		void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

void PointFilter::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	ROS_INFO("recieved points");
	pcl::PCLPointCloud2 pcl_pc;
	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, pcl_pc);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromPCLPointCloud2(pcl_pc, *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Perform the actual filtering 
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.2, 10.0);
	pass.filter(*cloud_filtered);

	filtered_pub.publish(*cloud_filtered);

}

PointFilter::PointFilter(ros::NodeHandle input_handler){
	nh = input_handler;
	filtered_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/head/kinect2/qhd/points/filtered_points", 1);
	string subscribe_topic = "/head/kinect2/qhd/points";
	cloud_sub = nh.subscribe(subscribe_topic, 1, &PointFilter::cloud_callback, this);
	ROS_INFO("Subscribed to node");
}



int main(int argc, char **argv){
	ros::init(argc, argv, "pointCloudFilter");
	ROS_INFO("init");
	ros::NodeHandle input_handler;
	PointFilter pf(input_handler);
	ros::spin();
}

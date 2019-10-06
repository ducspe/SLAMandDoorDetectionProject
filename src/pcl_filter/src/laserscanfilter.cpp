#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>

#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>  
#include <pcl/io/pcd_io.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub_merge;
bool IsZero (int i) { return i==0; }
bool IsHundered (float i) { return i==100.0; }


void processFirstDerivitive(sensor_msgs::LaserScan& scan){
	float min_angle = scan.angle_min;
	float max_angle = scan.angle_max;      
	float inc = scan.angle_increment;
	int length = (min_angle-max_angle)/inc;

	std::vector<float> filtered_scan;


	float current_angle = min_angle+inc;

	filtered_scan.push_back(scan.ranges[0] - scan.ranges[1]);

	int index=1;
	while (current_angle < max_angle-inc){
		float diff = scan.ranges[index-1] - scan.ranges[index+1];
		filtered_scan.push_back(diff);
		current_angle += inc;
		index++;
	}

	filtered_scan.push_back(scan.ranges[index-1] - scan.ranges[index]);


	
  	sensor_msgs::LaserScan output;
	output.header = scan.header;
	output.ranges = filtered_scan;
	output.angle_min = min_angle;
	output.angle_max = max_angle;
	output.angle_increment = inc;
	output.time_increment = scan.time_increment;
	output.scan_time = scan.scan_time;
	output.range_min = scan.range_min;
	output.range_max = scan.range_max;

	pub2.publish(output);

}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
	float min_angle = scan->angle_min;
	float max_angle = scan->angle_max;      
	float inc = scan->angle_increment;
	int length = (min_angle-max_angle)/inc;

	std::vector<float> filtered_scan;


	float current_angle = min_angle;
	int window_size = 10;
	int index = window_size;

	int i=0;
	for (;i<window_size; i++){
		//filtered_scan.push_back(scan->ranges[i]);
		filtered_scan.push_back(nanf(""));
	}

	std::vector<float> tmp_array;
	while (current_angle < max_angle-inc*window_size){
		tmp_array.clear();
		tmp_array.insert(tmp_array.begin(),scan->ranges.begin() +index-window_size, scan->ranges.begin()+index+window_size);
		std::replace_if (tmp_array.begin(), tmp_array.end(), isnan,100);
		
		std::sort(tmp_array.begin(), tmp_array.end());
		filtered_scan.push_back(tmp_array[window_size+1]);

		index++;
		current_angle += inc;
		i++;

	}

	int offset = i;
	for (;i<offset+window_size; i++){
		//filtered_scan.push_back(scan->ranges[i]);
		filtered_scan.push_back(nanf(""));
	}

	//std::replace_if (filtered_scan.begin(), filtered_scan.end(), IsZero, nanf(""));
	std::replace_if (filtered_scan.begin(), filtered_scan.end(), IsHundered,  scan->range_max-0.1);

	
  	sensor_msgs::LaserScan output;
	output.header = scan->header;
	output.ranges = filtered_scan;
	output.angle_min = min_angle;
	output.angle_max = max_angle;
	output.angle_increment = inc;
	output.time_increment = scan->time_increment;
	output.scan_time = scan->scan_time;
	output.range_min = scan->range_min;
	output.range_max = scan->range_max;

	processFirstDerivitive(output);
	pub.publish(output);

}
 
ros::Publisher pub3;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 // Container for original & filtered data
  pcl::PCLPointCloud2* input = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(input);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *input);

  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setMeanK(40);
  sor.setStddevMulThresh(1.0);
  sor.filter(*input);


  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*input, output);

  sensor_msgs::Image image; 
  pcl::toROSMsg (output, image); 

  pub3.publish(image);
}


using namespace sensor_msgs;
using namespace message_filters;

void mergeLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::LaserScan::ConstPtr& scan2){

	float min_angle = scan->angle_min;
	float max_angle = scan->angle_max;      
	float inc = scan->angle_increment;
	int length = (min_angle-max_angle)/inc;

	std::vector<float> filtered_scan;


	float current_angle = min_angle;
	int i=0;
	while(current_angle< max_angle){
		filtered_scan.push_back(std::min(scan->ranges[i], scan2->ranges[i]));
		current_angle += inc;
		i++;
	}


	
  	sensor_msgs::LaserScan output;
	output.header = scan->header;
	output.ranges = filtered_scan;
	output.angle_min = min_angle;
	output.angle_max = max_angle;
	output.angle_increment = inc;
	output.time_increment = scan->time_increment;
	output.scan_time = scan->scan_time;
	output.range_min = scan->range_min;
	output.range_max = scan->range_max;


	pub_merge.publish(output);

}
 



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "scan_filter_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("input_scan", 1, processLaserScan);

  pub = nh.advertise<sensor_msgs::LaserScan> ("output_scan", 1);
  pub2 = nh.advertise<sensor_msgs::LaserScan> ("output_scan_div", 1);


  //ros::Subscriber sub3 = nh.subscribe ("input_cloud", 1, cloud_cb);    
  //pub3 = nh.advertise<sensor_msgs::Image> ("output_cloud", 30);


  pub_merge = nh.advertise<sensor_msgs::LaserScan> ("output_scan_merge", 1);


  message_filters::Subscriber<LaserScan> scan1_sub(nh, "scan_merge_input1", 1);
  message_filters::Subscriber<LaserScan> scan2_sub(nh, "scan_merge_input2", 1);
  TimeSynchronizer<LaserScan, LaserScan> sync(scan1_sub, scan2_sub, 10);
  sync.registerCallback(boost::bind(&mergeLaserScan, _1, _2));

  // Spin
  ros::spin ();
}

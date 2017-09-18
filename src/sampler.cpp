#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;
float x, y, z;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (x, y, z);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_sampler");
  ros::NodeHandle nh;

  nh.param("pointcloud_sampler/x", x, 1.0f);
  nh.param("pointcloud_sampler/y", y, 1.0f);
  nh.param("pointcloud_sampler/z", z, 1.0f);

  std::string in_topic, out_topic;
  nh.param("pointcloud_sampler/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered"));
  nh.param("pointcloud_sampler/output_topic", out_topic, std::string("pointcloud_sampler/pointcloud"));

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

  // Spin
  ros::spin ();
}
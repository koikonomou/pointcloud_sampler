#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher pub;
float x, y, z;
float zmin, z_threshold;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;


  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> flt;
  flt.setInputCloud (cloudPtr);
  flt.setLeafSize (x, y, z);
  flt.filter (cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered, *pcl);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*pcl).size(); i++)
  {
    pcl::PointXYZ pt(pcl->points[i].x, pcl->points[i].y, pcl->points[i].z);
    float zmin;
    if (abs(pt.z - zmin) < z_threshold) 
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(pcl);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*pcl);

  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(*pcl, cloud2);
  pcl_conversions::fromPCL(cloud2 , output);

  pub.publish (output);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "ground_filtered");
  ros::NodeHandle nh;

  nh.param("ground_filtered/x", x, 1.0f);
  nh.param("ground_filtered/y", y, 1.0f);
  nh.param("ground_filtered/z", z, 5.0f);
  nh.param("ground_filtered/zmin", zmin, 1.0f);
  nh.param("ground_filtered/z_threshold", z_threshold, 1.0f);

  std::string in_topic, out_topic;
  nh.param("ground_filtered/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered"));
  nh.param("ground_filtered/output_topic", out_topic, std::string("ground_filtered/pointcloud"));

  ros::Subscriber sub = nh.subscribe (in_topic, 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

  ros::spin ();
}
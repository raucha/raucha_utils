#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <boost/thread/thread.hpp>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>



using namespace std;

double xmax, xmin, ymax, ymin;
string ref_frame_id;
ros::Publisher pub;

nav_msgs::Odometry odom_out;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_XYZ;

void Callback(const sensor_msgs::PointCloud2ConstPtr& input) {
  ROS_DEBUG("get velodyne data");
  PointCloud_XYZ cloud;
  pcl::fromROSMsg(*input, cloud);

  static PointCloud_XYZ prev_cloud;
  if (0 == prev_cloud.size()) {
    ROS_INFO("got first PointClouds");
    prev_cloud = cloud;
    return;
  }
  ROS_INFO_STREAM("size  now:" << cloud.size() << " prev:" << prev_cloud.size());

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  PointCloud_XYZ filtered_cloud;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  // approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setLeafSize(1.0, 1.0, 1.0);
  approximate_voxel_filter.setInputCloud(cloud.makeShared());
  approximate_voxel_filter.filter(*filtered_cloud.makeShared());

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
//   ndt.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
//   ndt.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
//   ndt.setResolution(1.0);

  // Setting max number of registration iterations.
//   ndt.setMaximumIterations(35);

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud.makeShared());
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(prev_cloud.makeShared());
  Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0.0, 0.0, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  PointCloud_XYZ::Ptr useless_cloud(new PointCloud_XYZ);
//   ndt.align(*useless_cloud);
  ndt.align(*useless_cloud, init_guess);  
//   computeTransformation	(*useless_cloud);		
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  std::cout << "translation is " << ndt.getFinalTransformation() << std::endl;
  //  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  prev_cloud = cloud;

  // tring fid = scan->header.frame_id;
  // try {
  //   // tf::MessageFilterで待ってるから待ち時間は発生しないはずだけど一応
  //   listener->waitForTransform(
  //       ref_frame_id, fid,
  //       scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment),
  //       ros::Duration(1.0));
  // } catch (tf::TransformException& ex) {
  //   ROS_WARN("%s", ex.what());
  //   return;
  // }
  // sensor_msgs::PointCloud cloud;
  // static laser_geometry::LaserProjection projector_;
  // projector_.transformLaserScanToPointCloud(ref_frame_id, *scan, cloud, *listener);
  //
  // // pclに変換できた点のLaserScanでのインデックス配列が格納されている,channelsインデックス
  // int ch_i = -1;
  // for (int i = 0; i < cloud.channels.size(); i++) {
  //   if ("index" == cloud.channels[i].name) {
  //     ch_i = i;
  //     break;
  //   }
  // }
  // if (-1 == ch_i) {
  //   ROS_ERROR("There is not 'index' channel in PointCloud converted from LaserScan");
  // }
  //
  // // 四角形内のインデックスを記録
  // vector<int> rip_idx;
  // for (int i = 0; i < cloud.points.size(); i++) {
  //   float x = cloud.points.at(i).x;
  //   float y = cloud.points.at(i).y;
  //   if (xmin < x && x < xmax && ymin < y && y < ymax) {
  //     int tmp = (int)cloud.channels[ch_i].values[i];
  //     rip_idx.push_back(tmp);
  //   }
  // }
  //
  // // インデックスに登録された点を削除，無効な距離を設定
  // sensor_msgs::LaserScan scan_out = *scan;
  // for (vector<int>::iterator it = rip_idx.begin(); it < rip_idx.end(); ++it) {
  //   int i = (int)*it;
  //   scan_out.ranges.at(i) = scan_out.range_max + 1.0;
  // }
  //
  // pub.publish(scan_out);
}

int main(int argc, char** argv) {
  ROS_INFO("Hello World!");
  ros::init(argc, argv, "pc_scan_matcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // nh_private.param<double>("ymax", ymax, 1.0);
  nh_private.param<string>("frame_id", ref_frame_id, "velodyne");

  // tf::TransformListener tfl;
  // message_filters::Subscriber<sensor_msgs::LaserScan> scan_filter_sub_(nh, "velodyne_points", 5);
  // tf::MessageFilter<sensor_msgs::LaserScan> scan_filter_(scan_filter_sub_, tfl, ref_frame_id, 5);
  // scan_filter_.registerCallback(boost::bind(&laserCallback, _1, &tfl));

  pub = nh.advertise<sensor_msgs::LaserScan>("odom_out", 5);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, Callback);

  ros::spin();
}

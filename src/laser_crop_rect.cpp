#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

using namespace std;

double xmax, xmin, ymax, ymin;
string ref_frame_id;
ros::Publisher pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan, tf::TransformListener* listener) {
  ROS_DEBUG("get scan data");

  string fid = scan->header.frame_id;
  ROS_DEBUG_STREAM("scan->frame_id:" << fid << " reference frame_id:" << ref_frame_id);
  try {
    // LRFの最後の光線の時刻で変換できるようになるまで待つ
    bool is_timeout = !listener->waitForTransform(
        ref_frame_id, fid,
        scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment),
        ros::Duration(3.0));
    if(is_timeout){
      ROS_WARN("waitForTransform timeout");
      return;
    }
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  sensor_msgs::PointCloud cloud;
  static laser_geometry::LaserProjection projector_;
  try{
    projector_.transformLaserScanToPointCloud(ref_frame_id, *scan, cloud, *listener);
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // pclに変換できた点のLaserScanでのインデックス配列が格納されている,channelsインデックス
  int ch_i = -1;
  for (int i = 0; i < cloud.channels.size(); i++) {
    if ("index" == cloud.channels[i].name) {
      ch_i = i;
      break;
    }
  }
  if (-1 == ch_i) {
    ROS_ERROR("There is not 'index' channel in PointCloud converted from LaserScan");
  }

  // 四角形内のインデックスを記録
  vector<int> rip_idx;
  for (int i = 0; i < cloud.points.size(); i++) {
    float x = cloud.points.at(i).x;
    float y = cloud.points.at(i).y;
    if (xmin < x && x < xmax && ymin < y && y < ymax) {
      int tmp = (int)cloud.channels[ch_i].values[i];
      rip_idx.push_back(tmp);
    }
  }

  // インデックスに登録された点を削除，無効な距離を設定
  sensor_msgs::LaserScan scan_out = *scan;
  for (vector<int>::iterator it = rip_idx.begin(); it < rip_idx.end(); ++it) {
    int i = (int)*it;
    scan_out.ranges.at(i) = scan_out.range_max + 1.0;
  }

  pub.publish(scan_out);
}

int main(int argc, char** argv) {
  ROS_INFO("Hello World!");
  ros::init(argc, argv, "laser_crop_rect");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<double>("xmin", xmin, -1.0);
  nh_private.param<double>("xmax", xmax, 1.0);
  nh_private.param<double>("ymin", ymin, -1.0);
  nh_private.param<double>("ymax", ymax, 1.0);
  nh_private.param<string>("frame_id", ref_frame_id, "/base_link");
  ROS_INFO_STREAM("xmin:" << xmin << " xmax:" << xmax << " ymin:" << ymin << " ymax:" << ymax
                          << " reference frame_id:" << ref_frame_id);

  tf::TransformListener tfl;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_filter_sub_(nh, "scan_in", 5);
  tf::MessageFilter<sensor_msgs::LaserScan> scan_filter_(scan_filter_sub_, tfl, ref_frame_id, 5);
  scan_filter_.registerCallback(boost::bind(&laserCallback, _1, &tfl));

  pub = nh.advertise<sensor_msgs::LaserScan>("scan_out", 5);

  ros::spin();
}
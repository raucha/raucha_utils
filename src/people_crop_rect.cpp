#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <people_msgs/People.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

using namespace std;

double xmax, xmin, ymax, ymin;
string ref_frame_id;
ros::Publisher pub;

sensor_msgs::PointCloud people2cloud(const people_msgs::People::ConstPtr& arg){
  sensor_msgs::PointCloud ret;
  ret.header = arg->header;
  for(int i=0; i<arg->people.size(); i++){
    geometry_msgs::Point32 point;
    point.x = arg->people.at(i).position.x;
    point.y = arg->people.at(i).position.y;
    point.z = arg->people.at(i).position.z;
    ret.points.push_back(point);
  }
  return ret;
}

void callback(const people_msgs::People::ConstPtr& scan, tf::TransformListener* listener) {
  ROS_DEBUG("get people data");

  // tfが実行できるようになるまで待つ
  string fid = scan->header.frame_id;
  ROS_DEBUG_STREAM("scan->frame_id:" << fid << "   reference frame_id:" << ref_frame_id);
  try {
    bool is_timeout = !listener->waitForTransform(
        ref_frame_id, fid,
        scan->header.stamp,
        ros::Duration(3.0));
    if(is_timeout){
      ROS_WARN("waitForTransform timeout");
      return;
    }
  } catch (tf::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // People->PointClouに変換
  sensor_msgs::PointCloud rawCloud;
  rawCloud = people2cloud(scan);
  sensor_msgs::PointCloud cloud;
  // "ref_frame_id"中心の座標系に変換実行
  ROS_DEBUG_STREAM("scan->frame_id:" << rawCloud.header.frame_id << "   reference frame_id:" << ref_frame_id);
  listener->transformPointCloud(ref_frame_id, rawCloud, cloud);

  // 四角形外のインデックスを記録
  vector<int> vailed_idx;
  for (int i = 0; i < cloud.points.size(); i++) {
    float x = cloud.points.at(i).x;
    float y = cloud.points.at(i).y;
    if (x>xmin || xmax<x || y<ymin && ymax<y) {
      vailed_idx.push_back(i);
    }
  }

  // インデックスに登録された点を削除，無効な距離を設定
  people_msgs::People scan_out;
  scan_out.header = scan->header;
  for (vector<int>::iterator it = vailed_idx.begin(); it < vailed_idx.end(); ++it) {
    int i = (int)*it;
    scan_out.people.push_back(scan->people.at(i));
  }

  // 発行
  pub.publish(scan_out);
}

int main(int argc, char** argv) {
  ROS_INFO("Hello World!");
  ros::init(argc, argv, "people_crop_rect");
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
  message_filters::Subscriber<people_msgs::People> people_filter_sub_(nh, "people_in", 5);
  tf::MessageFilter<people_msgs::People> people_filter_(people_filter_sub_, tfl, ref_frame_id, 5);
  people_filter_.registerCallback(boost::bind(&callback, _1, &tfl));

  pub = nh.advertise<people_msgs::People>("people_out", 5);

  ros::spin();
}
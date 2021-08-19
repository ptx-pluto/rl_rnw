#include <ros/ros.h>

#include <string>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
using nav_msgs::OdometryConstPtr;

template<typename T>
T get_param_default( ros::NodeHandle & nh, string const & key, T const & default_val ){
  T val;
  if ( !nh.getParam(key,val) ) {
    val = default_val;
    ROS_ERROR_STREAM("Parameter \'" << key << "\' not found, using default value " << default_val << "!");
  }
  return val;
}

inline void publish_frame( Matrix3d const & R, Vector3d const & T, string const & name, string const & parent ){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(T.x(),T.y(),T.z()) );
  Eigen::Quaterniond quat(R);
  tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

inline Matrix3d pose2R( geometry_msgs::Pose const & pose ){
  auto const & quat = pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  return q.normalized().toRotationMatrix();
}

inline Vector3d pose2T( geometry_msgs::Pose const & pose ){
  auto const & pos = pose.position;
  return { pos.x, pos.y, pos.z };
}

inline Vector3d odom2T( OdometryConstPtr const & odom ){
  return pose2T(odom->pose.pose);
}

inline Matrix3d odom2R( OdometryConstPtr const & odom ){
  return pose2R(odom->pose.pose);
}

string tf_frame_name, tf_parent_name;

void on_odom( nav_msgs::OdometryConstPtr const & msg ){
  publish_frame(odom2R(msg),odom2T(msg),tf_frame_name,tf_parent_name);
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"odom2tf_node");

  ros::NodeHandle nh("~");

  tf_frame_name = get_param_default<string>(nh,"name","name");
  tf_parent_name = get_param_default<string>(nh,"parent","world");

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("odom",1,on_odom);

  ros::spin();

  ros::shutdown();

  return 0;

}
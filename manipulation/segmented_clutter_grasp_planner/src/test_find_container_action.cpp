#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_manipulation_msgs/FindContainerAction.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_find_container_action");

  if(argc < 2)
  {
    ROS_ERROR("Usage: test_find_container_action <point_cloud_topic>");
    exit(-1);
  }
  tf::TransformListener tfl;
  ros::Duration(1.0).sleep();

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<object_manipulation_msgs::FindContainerAction> ac("find_container_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  sensor_msgs::PointCloud2::ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(argv[1], ros::Duration(5.0));
  if(!cloud)
  {
    ROS_INFO("No message received on topic %s", argv[1]);
    exit(-2);
  }



  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action

  object_manipulation_msgs::FindContainerGoal goal;


  tfl.waitForTransform("/base_link", cloud->header.frame_id, cloud->header.stamp, ros::Duration(2.0));
  pcl_ros::transformPointCloud("/base_link", *cloud, goal.cloud, tfl);

  //# x: 0 - 0.5 y: -0.3 to -0.7  z: 0.22 - 0.82
  goal.box_pose.header.frame_id = "/base_link";
  goal.box_pose.pose.position.x = 0.25;
  goal.box_pose.pose.position.y = -0.5;
  goal.box_pose.pose.position.z = 0.53;
  goal.box_dims.x = 0.5;
  goal.box_dims.y = 0.4;
  goal.box_dims.z = 0.6;



  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

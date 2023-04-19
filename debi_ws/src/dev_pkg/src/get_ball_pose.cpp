#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_object_pose_node");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::Publisher obj1_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/obj1_pose", 10);
  ros::Publisher obj2_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/obj2_pose", 10);

  gazebo_msgs::GetModelState srv1;
  srv1.request.model_name = "red_ball";

  gazebo_msgs::GetModelState srv2;
  srv2.request.model_name = "blue_ball";

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    if (client.call(srv1))
    {
      geometry_msgs::PoseStamped obj1_pose_msg;
      obj1_pose_msg.header.stamp = ros::Time::now();
      obj1_pose_msg.header.frame_id = "map";
      obj1_pose_msg.pose = srv1.response.pose;

      obj1_pose_pub.publish(obj1_pose_msg);
    }
    else
    {
      ROS_ERROR("Failed to call service /gazebo/get_model_state for object 1");
    }

    if (client.call(srv2))
    {
      geometry_msgs::PoseStamped obj2_pose_msg;
      obj2_pose_msg.header.stamp = ros::Time::now();
      obj2_pose_msg.header.frame_id = "map";
      obj2_pose_msg.pose = srv2.response.pose;

      obj2_pose_pub.publish(obj2_pose_msg);
    }
    else
    {
      ROS_ERROR("Failed to call service /gazebo/get_model_state for object 2");
    }

    rate.sleep();
  }

  return 0;
}
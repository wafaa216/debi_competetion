#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void obj1PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Received object 1 pose: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  geometry_msgs::PoseStamped obj1_pose_msg;
  obj1_pose_msg.header.frame_id = "map";
  obj1_pose_msg.pose.position.x = msg->pose.position.x - 0.5;
  obj1_pose_msg.pose.position.y = msg->pose.position.y;
  obj1_pose_msg.pose.position.z = msg->pose.position.z;
  obj1_pose_msg.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = obj1_pose_msg.header.frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = obj1_pose_msg.pose;

  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Reached object 1 goal!");
    goal.target_pose.pose.position.x = msg->pose.position.x + 0.3;
    ac.sendGoal(goal);
  }
  else
  {
    ROS_ERROR("Failed to reach object 1 goal");
  }
}

void obj2PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Received object 2 pose: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  geometry_msgs::PoseStamped obj2_pose_msg;
  obj2_pose_msg.header.frame_id = "map";
  obj2_pose_msg.pose.position.x = msg->pose.position.x - 0.5;
  obj2_pose_msg.pose.position.y = msg->pose.position.y;
  obj2_pose_msg.pose.position.z = msg->pose.position.z;
  obj2_pose_msg.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = obj2_pose_msg.header.frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = obj2_pose_msg.pose;

  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Reached object 2 goal!");
    goal.target_pose.pose.position.x = msg->pose.position.x + 0.3;
    ac.sendGoal(goal);
  }
  else
  {
    ROS_ERROR("Failed to reach object 2 goal");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_goal");
  ros::NodeHandle nh;

  ros::Subscriber obj1_pose_sub = nh.subscribe("/obj1_pose", 10, obj1PoseCallback);
  sleep(3);

  ros::Subscriber obj2_pose_sub = nh.subscribe("/obj2_pose", 10, obj2PoseCallback);
  sleep(3);

  ros::spin();

  return 0;
}
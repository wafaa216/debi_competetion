#include<iostream>
using namespace std ; 
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "joint_move.h"
using std::vector;

static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";


class points_group {
    public :

    void arm_go_to_point(string point_name){
        moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        
        move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues(point_name));    
        bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO_NAMED(point_name, "Visualiz<ing plan 1 (pose goal) %s", success ? "" : "FAILED");
        move_group_interface_arm.move();
        
        if (success == 1 ) success = "successed" ; 
        cout<< endl<<"______________ robot reached "<< point_name << " point status "<< success << "______________ "<< endl<< endl ; 
    }


    void arm_go_to_xyz_point(vector <double> coordinates){
        moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
       
        geometry_msgs::PoseStamped current_pose;
        current_pose = move_group_interface_arm.getCurrentPose("end_effector_link");
        cout<<coordinates[0]<<coordinates[1]<<coordinates[2]<<endl;

       // this to create a point in the space and let the robot go from its current pose to the new
       // target pose, here the point is at the center of the BBox 
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation = current_pose.pose.orientation;
        target_pose1.position.x = coordinates[0]-0.3; 
        // target_pose1.position.y = current_pose.pose.position.y;
        target_pose1.position.y = coordinates[1]; //current_pose.pose.position.y;
        target_pose1.position.z = coordinates[2]+ 0.3;
        move_group_interface_arm.setPoseTarget(target_pose1);

        bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("motion", "Goal position x:%f y:%f z:%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
        ROS_INFO_NAMED("motion", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        move_group_interface_arm.move();
      }


    void gripper_go_to_point(string state){
        moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        
        move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues(state));    
        bool success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        ROS_INFO_NAMED(state, "Visualiz<ing plan 1 (pose goal) %s", success ? "" : "FAILED");
        move_group_interface_gripper.move();
       
        while (success == 0 ) {move_group_interface_gripper.move();} ; 
        cout<< endl<<"______________ gripper is  "<< state << "  status "<< success << "______________ "<< endl<< endl ; 
    }


    void go_to_a_usr_defined_point(std::vector<double> joints){
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        
    joint_movement joint_move ;  
    joints = joint_move.get_rad_from_deg(joints);
    // joint_move.get_deg_from_rad(joint_group_positions); // just to visualize // get current position angles

    move_group_interface_arm.setJointValueTarget(joints);
    move_group_interface_arm.move();

    }
} ;
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "openni_tf_listener");

  ros::NodeHandle node;

  // publisher declaration
  ros::Publisher neck_joint = node.advertise<geometry_msgs::Point>("neck_joint", 1);
  ros::Publisher head_joint = node.advertise<geometry_msgs::Point>("head_joint", 1);
  ros::Publisher torso_joint = node.advertise<geometry_msgs::Point>("torso_joint", 1);
  ros::Publisher left_shoulder_joint = node.advertise<geometry_msgs::Point>("left_shoulder_joint", 1);
  ros::Publisher left_elbow_joint = node.advertise<geometry_msgs::Point>("left_elbow_joint", 1);
  ros::Publisher left_hand_joint = node.advertise<geometry_msgs::Point>("left_hand_joint", 1);
  ros::Publisher right_shoulder_joint = node.advertise<geometry_msgs::Point>("right_shoulder_joint", 1);
  ros::Publisher right_elbow_joint = node.advertise<geometry_msgs::Point>("right_elbow_joint", 1);
  ros::Publisher right_hand_joint = node.advertise<geometry_msgs::Point>("right_hand_joint", 1);
  ros::Publisher left_hip_joint = node.advertise<geometry_msgs::Point>("left_hip_joint", 1);
  ros::Publisher left_knee_joint = node.advertise<geometry_msgs::Point>("left_knee_joint", 1);
  ros::Publisher left_foot_joint = node.advertise<geometry_msgs::Point>("left_foot_joint", 1);
  ros::Publisher right_hip_joint = node.advertise<geometry_msgs::Point>("right_hip_joint", 1);
  ros::Publisher right_knee_joint = node.advertise<geometry_msgs::Point>("right_knee_joint", 1);
  ros::Publisher right_foot_joint = node.advertise<geometry_msgs::Point>("right_foot_joint", 1);
  ros::Publisher human_position = node.advertise<geometry_msgs::Point>("human_position", 1);

  // listener 
  tf::TransformListener listener;

  ros::Rate rate(50.0); // frequency of operation

  while (node.ok())
  {
      // Transforms declared for each joint
      tf::StampedTransform transform_neck, transform_head, transform_torso, 
                              transform_left_shoulder, transform_left_elbow, transform_left_hand, 
                                  transform_right_shoulder, transform_right_elbow, transform_right_hand, 
                                      transform_left_hip, transform_left_knee, transform_left_foot,
                                          transform_right_hip, transform_right_knee, transform_right_foot;
      try
      {
          // each joint frame to reference frame transforms 
          listener.lookupTransform("/neck_1", "/openni_depth_frame",ros::Time(0), transform_neck);
          listener.lookupTransform("/head_1", "/openni_depth_frame",ros::Time(0), transform_head);
          listener.lookupTransform("/torso_1", "/openni_depth_frame",ros::Time(0), transform_torso);
          listener.lookupTransform("/left_shoulder_1", "/openni_depth_frame",ros::Time(0), transform_left_shoulder);
          listener.lookupTransform("/left_elbow_1", "/openni_depth_frame",ros::Time(0), transform_left_elbow);
          listener.lookupTransform("/left_hand_1", "/openni_depth_frame",ros::Time(0), transform_left_hand);
          listener.lookupTransform("/right_shoulder_1", "/openni_depth_frame",ros::Time(0), transform_right_shoulder);
          listener.lookupTransform("/right_elbow_1", "/openni_depth_frame",ros::Time(0), transform_right_elbow);
          listener.lookupTransform("/right_hand_1", "/openni_depth_frame",ros::Time(0), transform_right_hand);
          listener.lookupTransform("/left_hip_1", "/openni_depth_frame",ros::Time(0), transform_left_hip);
          listener.lookupTransform("/left_knee_1", "/openni_depth_frame",ros::Time(0), transform_left_knee);
          listener.lookupTransform("/left_foot_1", "/openni_depth_frame",ros::Time(0), transform_left_foot);
          listener.lookupTransform("/right_hip_1", "/openni_depth_frame",ros::Time(0), transform_right_hip);
          listener.lookupTransform("/right_knee_1", "/openni_depth_frame",ros::Time(0), transform_right_knee);
          listener.lookupTransform("/right_foot_1", "/openni_depth_frame",ros::Time(0), transform_right_foot);

      }
          catch (tf::TransformException &ex) 
      {
          //ROS_ERROR("%s",ex.what());
          ros::Duration(0.10).sleep();
          continue;
      }

      // geometry points declaration for storing 3D coordinates of joints and then published later 
      geometry_msgs::Point neck_pose, head_pose, torso_pose, 
                              left_shoulder_pose, left_elbow_pose, left_hand_pose,
                                  right_shoulder_pose, right_elbow_pose, right_hand_pose, 
                                      left_hip_pose, left_knee_pose, left_foot_pose, 
                                          right_hip_pose, right_knee_pose, right_foot_pose, human_pose;

      // joint position extraction and store
      // neck joint                                   
      neck_pose.x = transform_neck.getOrigin().x();
      neck_pose.y = transform_neck.getOrigin().y();
      // neck_pose.z = transform_neck.getOrigin().z();
      
      // head joint
      head_pose.x = transform_head.getOrigin().x();
      head_pose.y = transform_head.getOrigin().y();
      // head_pose.z = transform_head.getOrigin().z();
      
      // torso joint
      torso_pose.x = transform_torso.getOrigin().x();
      torso_pose.y = transform_torso.getOrigin().y();
      // torso_pose.z = transform_torso.getOrigin().z();
      
      // left shoulder joint 
      left_shoulder_pose.x = transform_left_shoulder.getOrigin().x();
      left_shoulder_pose.y = transform_left_shoulder.getOrigin().y();
      // left_shoulder_pose.z = transform_left_shoulder.getOrigin().z();
      
      // left elbow joint
      left_elbow_pose.x = transform_left_elbow.getOrigin().x();
      left_elbow_pose.y = transform_left_elbow.getOrigin().y();
      // left_elbow_pose.z = transform_left_elbow.getOrigin().z();
      
      // left hand joint
      left_hand_pose.x = transform_left_hand.getOrigin().x();
      left_hand_pose.y = transform_left_hand.getOrigin().y();
      // left_hand_pose.z = transform_left_hand.getOrigin().z();
      
      // right shoulder joint
      right_shoulder_pose.x = transform_right_shoulder.getOrigin().x();
      right_shoulder_pose.y = transform_right_shoulder.getOrigin().y();
      // right_shoulder_pose.z = transform_right_shoulder.getOrigin().z();
      
      // right elbow joint
      right_elbow_pose.x = transform_right_elbow.getOrigin().x();
      right_elbow_pose.y = transform_right_elbow.getOrigin().y();
      // right_elbow_pose.z = transform_right_elbow.getOrigin().z();
      
      // right hand joint
      right_hand_pose.x = transform_right_hand.getOrigin().x();
      right_hand_pose.y = transform_right_hand.getOrigin().y();
      // right_hand_pose.z = transform_right_hand.getOrigin().z();
      
      // left hip joint
      left_hip_pose.x = transform_left_hip.getOrigin().x();
      left_hip_pose.y = transform_left_hip.getOrigin().y();
      // left_hip_pose.z = transform_left_hip.getOrigin().z();
      
      // left knee joint
      left_knee_pose.x = transform_left_knee.getOrigin().x();
      left_knee_pose.y = transform_left_knee.getOrigin().y();
      // left_knee_pose.z = transform_left_knee.getOrigin().z();
      
      // left foot joint
      left_foot_pose.x = transform_left_foot.getOrigin().x();
      left_foot_pose.y = transform_left_foot.getOrigin().y();
      // left_foot_pose.z = transform_left_foot.getOrigin().z();
      
      // right hip joint
      right_hip_pose.x = transform_right_hip.getOrigin().x();
      right_hip_pose.y = transform_right_hip.getOrigin().y();
      // right_hip_pose.z = transform_right_hip.getOrigin().z();
      
      // right knee joint
      right_knee_pose.x = transform_right_knee.getOrigin().x();
      right_knee_pose.y = transform_right_knee.getOrigin().y();
      // right_knee_pose.z = transform_right_knee.getOrigin().z();
      
      // right foot joint
      right_foot_pose.x = transform_right_foot.getOrigin().x();
      right_foot_pose.y = transform_right_foot.getOrigin().y();
      // right_foot_pose.z = transform_right_foot.getOrigin().z();

      // human pose
      human_pose.x = (neck_pose.x + head_pose.x + torso_pose.x + left_shoulder_pose.x + left_elbow_pose.x + left_hand_pose.x + left_hip_pose.x + left_knee_pose.x + left_foot_pose.x
       + right_shoulder_pose.x + right_elbow_pose.x + right_hand_pose.x + right_hip_pose.x + right_knee_pose.x + right_foot_pose.x)/15;
      human_pose.y = (neck_pose.y + head_pose.y + torso_pose.y + left_shoulder_pose.y + left_elbow_pose.y + left_hand_pose.y + left_hip_pose.y + left_knee_pose.y + left_foot_pose.y
       + right_shoulder_pose.y + right_elbow_pose.y + right_hand_pose.y + right_hip_pose.y + right_knee_pose.y + right_foot_pose.y)/15;
      human_pose.z = 0;

      // joint positions publish
      neck_joint.publish(neck_pose);
      head_joint.publish(head_pose);
      torso_joint.publish(torso_pose);
      left_shoulder_joint.publish(left_shoulder_pose);
      left_elbow_joint.publish(left_elbow_pose);
      left_hand_joint.publish(left_hand_pose);
      right_shoulder_joint.publish(right_shoulder_pose);
      right_elbow_joint.publish(right_elbow_pose);
      right_hand_joint.publish(right_hand_pose);
      left_hip_joint.publish(left_hip_pose);
      left_knee_joint.publish(left_knee_pose);
      left_foot_joint.publish(left_foot_pose);
      right_hip_joint.publish(right_hip_pose);
      right_knee_joint.publish(right_knee_pose);
      right_foot_joint.publish(right_foot_pose);
      human_position.publish(human_pose);

      rate.sleep();
  }
  return 0;
};
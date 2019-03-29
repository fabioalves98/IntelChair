
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <math.h>



double ax;
double ay;
double az;

double mx;
double my;
double mz;

double roll;
double pitch;
double yaw;

void imuAccelCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // ROS_INFO("%f", msg->linear_acceleration.z);

    double accelx = msg->linear_acceleration.x;
    double accely = msg->linear_acceleration.y; 
    double accelz = msg->linear_acceleration.z;

    // Calc pitch roll and yaw
    // pitch = 180 * atan2(accelx, sqrt(accely*accely + accelz*accelz))/M_PI;
    // roll = 180 * atan2(accely, sqrt(accelx*accelx + accelz*accelz))/M_PI; 
    pitch = atan2(accelx, sqrt(accely*accely + accelz*accelz));
    roll = atan2(accely, sqrt(accelx*accelx + accelz*accelz)); 

    double mag_x = mx*cos(pitch) + my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch);
    double mag_y = my * cos(roll) - mz * sin(roll);
    yaw = atan2(-mag_y, mag_x);
    double yaw_deg = 180 * yaw /M_PI;
    // ROS_INFO("Yaw degrees: %f", yaw);

    // Projetar valores de aceleração visto que existe inclinação do IMU!
    ax = accelx + 3.35;
    // ROS_INFO("Accelx: %f\nPitch: %f", accelx, sin(30));
    ROS_INFO("Ax: %f", ax);

}
void imuMagCallback(const sensor_msgs::MagneticField::ConstPtr& msg){
    // ROS_INFO("%f", msg->magnetic_field.x);
    mx = msg->magnetic_field.x;
    my = msg->magnetic_field.y;
    mz = msg->magnetic_field.z;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1000, imuAccelCallback);
    ros::Subscriber mag_sub = n.subscribe("/imu/mag", 1000, imuMagCallback);

    double x = 0;
    double y = 0;
    double th = 0;

    double vx;
    double vy;
    double vth;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

   

    ros::Rate r(100); 

    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();


        // ROS_INFO("\nPITCH: %f\nROLL: %f\nYAW: %f", pitch, roll, yaw);

       

        // Calc vx based off of accel and delta t
        double dt = (current_time - last_time).toSec();
        vx += (ax * dt);
        vy += (ay * dt);
        vth = yaw;

        // ROS_INFO("\nVX: %f\nVy: %f\n", vx, vy);

        // Compute odometry
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += (vx * dt);
        y += (vy * dt);
        th += vth;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        // Publishing the transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // Publishing odom topic
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();

    }

    ros::spin();

}
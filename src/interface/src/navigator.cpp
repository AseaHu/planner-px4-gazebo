#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define pi 3.14159265358979
ros::Publisher odom_pub, cam_pub;
Eigen::Matrix3d R;

nav_msgs::Odometry odom;
geometry_msgs::PoseStamped body, camera;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;

    camera.header.stamp = ros::Time::now();
    camera.pose.position = odom.pose.pose.position;
    Eigen::Quaterniond q_odom, q_cam;
    q_odom.w() = odom.pose.pose.orientation.w;
    q_odom.x() = odom.pose.pose.orientation.x;
    q_odom.y() = odom.pose.pose.orientation.y;
    q_odom.z() = odom.pose.pose.orientation.z;

    q_cam = q_odom * R;

    camera.pose.orientation.w = q_cam.w();
    camera.pose.orientation.x = q_cam.x();
    camera.pose.orientation.y = q_cam.y();
    camera.pose.orientation.z = q_cam.z();

    odom_pub.publish(odom);
    cam_pub.publish(camera);
    std::cout<<"pose has been published."<<std::endl;
}


void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    geometry_msgs::Pose real_pose;
    int n = msg->pose.size();

    real_pose = msg->pose[n-1];

    camera.header.stamp = ros::Time::now();
    body.pose.position = real_pose.position;
    body.pose.orientation = real_pose.orientation;

    Eigen::Quaterniond q_odom, q_cam;   
    q_odom.w() = real_pose.orientation.w;
    q_odom.x() = real_pose.orientation.x;
    q_odom.y() = real_pose.orientation.y;
    q_odom.z() = real_pose.orientation.z;

    Eigen::Vector3d tran = q_odom * Eigen::Vector3d(0.1, 0, 0);
    camera.pose.position.x = real_pose.position.x + tran.x();
    camera.pose.position.y = real_pose.position.y + tran.y();
    camera.pose.position.z = real_pose.position.z + tran.z();
    odom.pose.pose.position = real_pose.position;
    odom.pose.pose.orientation = real_pose.orientation;

    q_cam = q_odom * R;

    camera.pose.orientation.w = q_cam.w();
    camera.pose.orientation.x = q_cam.x();
    camera.pose.orientation.y = q_cam.y();
    camera.pose.orientation.z = q_cam.z();

    odom_pub.publish(odom);
    cam_pub.publish(camera);
    //std::cout<<"pose has been published."<<std::endl;
    std::cout<<real_pose.position.x<<"  "<<real_pose.position.y<<"  "<<real_pose.position.z<<std::endl;
}


int main(int argc, char **argv)
{
    R<< 0,      0,      1,      //camera to uav-body
        -1,     0,      0,
        0,      -1,     0;

    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;

    //ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1000, odom_cb);
    ros::Subscriber sub_gz = nh.subscribe <gazebo_msgs::ModelStates> ("/gazebo/model_states", 100, model_state_cb);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/vehicle_pose", 10);
    cam_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1000);

    ros::Rate rate(20.0);
    
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

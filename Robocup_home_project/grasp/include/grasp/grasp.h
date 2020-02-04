#ifndef GRASP_H
#define GRASP_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <tf_lookup/lookupTransform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <perception_msgs/Rect.h>

 // Std C++ headers
#include <string>
#include <vector>
#include <map>
// Eigen include
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
// tf include
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace Eigen;

namespace GRASP{

class grasp
{
private:
    ros::Subscriber sub_position;
    ros::Subscriber sub_position2;
    ros::Subscriber sub_transformation;
    ros::Subscriber sub_rotation;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    ros::Publisher pub;
    ros::NodeHandle n;

    geometry_msgs::PoseStamped  goal_pose;
    Vector3d target;
    Vector3d original;
    Vector3d translation_;
    Matrix3d rotation_;
    Quaterniond q;

    bool grasp_f(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
    bool grasp_rectify(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
    void get_desired_pose(const geometry_msgs::Point::ConstPtr& msg);
    void get_transformation(const geometry_msgs::Point::ConstPtr& msg);
    void get_rotation(const geometry_msgs::Quaternion::ConstPtr& msg);


public:
    grasp(ros::NodeHandle n): n(n){

    sub_transformation=n.subscribe<geometry_msgs::Point>("/translation",100,&grasp::get_transformation,this);
    sub_rotation=n.subscribe<geometry_msgs::Quaternion>("/rotation",100,&grasp::get_rotation,this);
    sub_position=n.subscribe<geometry_msgs::Point>("/Point3D",100,&grasp::get_desired_pose,this);
    service = n.advertiseService("/move_1",&grasp::grasp_f,this);
    service2 = n.advertiseService("/move_rectify",&grasp::grasp_rectify,this);
    ROS_INFO("You have got 3d point");
    
    };
};
}
#endif

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <iostream>

#include <image_geometry/pinhole_camera_model.h>

// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <perception_msgs/Rect.h>
#include <perception_msgs/arrived.h>
#include <wit_ros/ListenAndInterpret.h>

using namespace std;

class ObjectRecognition
{
private:
  ros::NodeHandle n;
  ros::Publisher pub;
  std::string target_object;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  wit_ros::ListenAndInterpret srv;
  ros::ServiceClient client_1;
  bool bekommen = 0;
  perception_msgs::Rect percep_msg;
  void callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
  void get_success(const perception_msgs::arrivedConstPtr& msg);
  void get_target_object(const std_msgs::StringConstPtr& msg);
  
public:
  ObjectRecognition(ros::NodeHandle n): n(n){

  // ros::Publisher pub_success = n.advertise<navigation_tiago::arrived>("/arrived", 100);

  sub2 = n.subscribe<perception_msgs::arrived>("/arrived",100,&ObjectRecognition::get_success,this);
  
  pub = n.advertise<perception_msgs::Rect>("rect",100);
  sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&ObjectRecognition::callback,this);
  sub3 = n.subscribe<std_msgs::String>("object",10,&ObjectRecognition::get_target_object,this);
  // client_1 = n.serviceClient<wit_ros::ListenAndInterpret>("/wit/listen_interpret");
  
  ROS_INFO_STREAM("you are detecting");
  ROS_INFO_STREAM("you finished detecting");
  

  };
  string current_name;
  string speaktext;

};

void ObjectRecognition::get_target_object(const std_msgs::StringConstPtr& msg)
{
  target_object = msg->data.c_str();
  sub3.shutdown();
}
void ObjectRecognition::get_success(const perception_msgs::arrivedConstPtr& msg)
{
  bekommen = msg->x;
  sub2.shutdown();
}

void ObjectRecognition::callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
  if (bekommen==(bool) 1)
  {
    for(int i=0; i<msg->bounding_boxes.size();i++)
    {
      if (msg->bounding_boxes[i].Class==target_object)
      {
        percep_msg.x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax)/2;
        percep_msg.y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax)/2;
        percep_msg.height = (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin);
        percep_msg.width = (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);

        pub.publish(percep_msg);
        // ROS_INFO_STREAM("publish target object position");
      }
    }
  }
}

int main(int argc,char **argv)
{
  
  ros::init(argc,argv,"object");
  ros::NodeHandle n;
  ObjectRecognition node(n);
  ros::spin();
  
  return 0;
}



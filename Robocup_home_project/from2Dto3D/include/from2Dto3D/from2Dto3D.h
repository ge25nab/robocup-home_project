#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

//#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/Point.h>
#include <perception_msgs/Rect.h>


using namespace std;
using namespace cv;


class From2Dto3D
{

    private:

      ros::NodeHandle nh_;
      ros::NodeHandle priv_nh_;

      ros::Publisher pub_pc_;
      ros::Subscriber sub_pc_;
      ros::Subscriber sub_rec_;

      pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
      geometry_msgs::Point msg;
      tf::TransformListener listener_;
      
      float c_x, o_x;
      float c_y, o_y;
      int position,position_al;

      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      void processRect(const perception_msgs::RectConstPtr & r);


    public:

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        sub_pc_=nh_.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points",1,&From2Dto3D::processCloud,this);
        sub_rec_=nh_.subscribe<perception_msgs::Rect>("/rect",1,&From2Dto3D::processRect,this);
        pub_pc_ = nh_.advertise<geometry_msgs::Point>("/Point3D",100);
        
        c_x = 0;
        c_y = 0;
        position = 0;
        position_al =0;
        o_x = 0;
        o_y = 0;

        ROS_INFO("from2Dto3D initialized ...");

      }

      ~From2Dto3D() {}
};

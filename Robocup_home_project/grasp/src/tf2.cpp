#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_lookup/lookupTransform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


int main(int argc,char **argv)
{
    
    ros::init(argc,argv,"tf2");

    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Publisher pub;
    ros::Publisher pub2;

    ros::Rate r(100);

    geometry_msgs::Quaternion q;
    geometry_msgs::Point translation;

    ros::Duration(20).sleep();

    client = n.serviceClient<tf_lookup::lookupTransform>("/lookupTransform");
    pub = n.advertise<geometry_msgs::Quaternion>("/rotation",10);
    pub2 = n.advertise<geometry_msgs::Point>("/translation",10);

    tf_lookup::lookupTransform srv;

    srv.request.target_frame = "/base_footprint";
    srv.request.source_frame = "/xtion_rgb_optical_frame";
    ros::Time x = ros::Time(0);
    srv.request.transform_time = x;

    if(client.call(srv))
    {
        ROS_INFO("get transform from head to foot");

        q.x = srv.response.transform.transform.rotation.x;
        q.y = srv.response.transform.transform.rotation.y;
        q.z = srv.response.transform.transform.rotation.z;
        q.w = srv.response.transform.transform.rotation.w;

        translation.x = srv.response.transform.transform.translation.x;
        translation.y = srv.response.transform.transform.translation.y;
        translation.z = srv.response.transform.transform.translation.z;
        
        ROS_INFO_STREAM("translation is x:"<<translation.x <<"y:"<<translation.y <<"z:"<<translation.z);
        ROS_INFO_STREAM("rotation matrix is: "<<q);

    }
    else
    {
        ROS_INFO("fail to call service lookupTransform!!!!!!!!!!!!!!!!!!!!!!!");
    }
    

  while (ros::ok())
  {
      pub.publish(q);
      pub2.publish(translation);

      ros::spinOnce();
      r.sleep();
  }

  return 0;
}
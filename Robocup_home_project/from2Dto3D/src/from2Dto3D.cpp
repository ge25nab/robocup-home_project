#include <from2Dto3D/from2Dto3D.h>

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{    

    pcl::fromROSMsg(*pc,pointcloud);
    position = pointcloud.width*c_y + c_x;
    position_al = pointcloud.width*o_y + o_x;
    if (!(isnan(pointcloud.points[position].x)||isnan(pointcloud.points[position].y)||isnan(pointcloud.points[position].z)))
    {

        msg.x=pointcloud.at(c_x,c_y).x;
        msg.y=pointcloud.at(c_x,c_y).y;
        msg.z=pointcloud.at(c_x,c_y).z;
    }   

}

void From2Dto3D::processRect(const perception_msgs::RectConstPtr& r)
{

    c_x = r->x;
    c_y = r->y;
    o_x = r->x - r->width/2;
    o_y = r->y - r->height/2;
       
    pub_pc_.publish(msg);
}



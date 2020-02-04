#include <navigation_tiago/navigation_tiago.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace NAVIGATIONTIAGO;


int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_tiago_node");
    ros::NodeHandle n;
    Navigation_tiago node(n);
    return 0;
}

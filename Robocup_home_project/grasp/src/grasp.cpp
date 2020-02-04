#include <grasp/grasp.h>

using namespace GRASP;
int main(int argc,char **argv)
{
  
  ros::init(argc,argv,"grasp");
//   ros::AsyncSpinner spinner(5);
//   spinner.start();
  ros::MultiThreadedSpinner spinner(4);
  ros::NodeHandle n;
  grasp node(n);
  spinner.spin();
  
  return 0;
}


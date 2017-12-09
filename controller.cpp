
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_filter/LandingField.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include "e_l_f_d.h"    //contents the configuration defines and different used typede


ros::Publisher VelocityPub;
ros::Publisher HeadingPub;
ros::Publisher TurnPub;






int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  //ros::Subscriber sub3 = n.subscribe("UAV_velocity",1,updateVelocity);
  //ros::Subscriber sub4 = n.subscribe("UAV_headingAngle",1,updateHeadingAngle);
// %EndTag(SUBSCRIBER)%


  // Create a ROS publisher for the output point cloud

  VelocityPub = n.advertise<std_msgs::Int32> ("UAV_velocity", 1);
  HeadingPub = n.advertise<std_msgs::Int32>("UAV_headingAngle", 1);
  TurnPub = n.advertise<std_msgs::Int32>("Turn_notification", 1);

  std_msgs::Int32 Speed;
  std_msgs::Int32 HeadingAngle;
  int Turn = 0;
  std_msgs::Int32 Turner;

  while(ros::ok()){
    cout << "new Speed: " ;
    cin >>  Speed.data;
    cout << "new HeadingAngle: ";
    cin >> HeadingAngle.data;
    cout << "Turn ? 0 = no, else = yes:" ;
    cin >> Turn;
    cout << endl;

    VelocityPub.publish(Speed);
    HeadingPub.publish(HeadingAngle);
    if(Turn != 0){
      TurnPub.publish(Turner);
  }


    ros::spinOnce();

  }



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}

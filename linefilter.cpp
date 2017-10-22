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

#include <pcl-1.7/pcl/io/pcd_io.h>

//specific include for the columns
#include "linerow.h"



float linePieceSize = 1.0;    //Size for one piece = 1m
int density = 200;            //points per 1 meter
int numberOfLines;

ros::Publisher pub;



void PCColumnHandler(const sensor_msgs::PointCloud2ConstPtr& input){

  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ> cloud_filteredConditional;

  // Covert to PCL data type
  pcl::fromROSMsg(*input, *unfiltered_cloud);

  // Perform the filtering
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (unfiltered_cloud);
//  sor.setLeafSize (0.1, 0.1,0);
//  sor.setMinimumPointsNumberPerVoxel (200);
//  sor.filter (cloud_filtered);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
  filter.setMinNeighborsInRadius(3);
  filter.setRadiusSearch(0.5);
  filter.setInputCloud(unfiltered_cloud);
  filter.filter(cloud_filtered);

  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -10.0)));

  numberOfLines = (int) (cloud_filterd.width/linePieceSize);
  line_t lineColunm[numberOfLInes];
  //Divide in defined pc pieces
  int i = 0;
  do{
  float xstart = cloud_filtered[i].x;
  pcl::PointXYZ begin = cloud_filterd[i];

  while(cloud_filterd[i] < xstart + linePieceSize){
    i ++;
  }
  pcl::PointXYZ end = cloud_filered[i];
  }
  while(i < cloud_filtered.width);


  // Convert to ROS data type
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(cloud_filteredConditional,output);


  // Publish the data.
  pub.publish (output);
}






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
  ros::init(argc, argv, "linefilter");

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
  ros::Subscriber sub = n.subscribe("point_cloud_unfiltered", 1, PCColumnHandler);
// %EndTag(SUBSCRIBER)%


  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_from_my_filter", 1);

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

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

#include <pcl-1.7/pcl/io/pcd_io.h>

//specific include for the columns
#include "linerow.h"

#define ISSIMUALTION 1


float linePieceSize = 1.0;    //Size for one piece = 1m
float maxDifference = 0.2;      //Max Difference which is accepted bevor a staff etc. is detected.
int density = 200;            //points per 1 meter
int numberOfLines;

ros::Publisher pub;
ros::Publisher linePub;       //Publisher for the lineCloumn


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
  pcl::fromROSMsg(*input, cloud_filtered);

  // Perform the filtering
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (unfiltered_cloud);
//  sor.setLeafSize (0.1, 0.1,0);
//  sor.setMinimumPointsNumberPerVoxel (200);
//  sor.filter (cloud_filtered);
//  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
//  filter.setMinNeighborsInRadius(3);
//  filter.setRadiusSearch(0.5);
//  filter.setInputCloud(unfiltered_cloud);
//  filter.filter(cloud_filtered);

//  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>);
//  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -10.0)));

  numberOfLines = (int) (abs((int)cloud_filtered[0].x) + abs((int)cloud_filtered[cloud_filtered.width-1].x))/linePieceSize;

  LineRow* TempRow =  new LineRow(numberOfLines, density, maxDifference);

  //Divide in defined pc pieces
  int i = 0;
  while(!(((int)cloud_filtered[i].x)%1 < 0.01 && ((int)cloud_filtered[i].x)%1 > -0.01)){ // Determinate the start point +- 1cm
    i ++;
  }
  do{
  int xstart = (int) cloud_filtered[i].x;
  //pcl::PointXYZ begin = cloud_filtered[i];
  pcl::PointCloud<pcl::PointXYZ> tempCloud;
  int j = 0;
  while(cloud_filtered[i].x < (xstart + linePieceSize)){
    tempCloud[j] = cloud_filtered[i];
    i ++;
  }

  TempRow->setNewLine(tempCloud,xstart);  //Make a new line with the given PC piece
  }
  while(i < cloud_filtered.width);


  // Convert to ROS data type
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(cloud_filteredConditional,output);

  //detemate if this is a Simulation -> publish lines, else store the made object into the data
  if(ISSIMUALTION){
    //@ToDo Convert into line pieces an publish it.

    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = linePieceSize;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_p tempArray = TempRow->getRowArray();

    geometry_msgs::Point p;
    p.z = 0;
    for(int i = 0; i < TempRow->getSize(); i++){
      p.x = tempArray[i].x;
      p.y = tempArray[i].q;

      line_list.points.push_back(p);
      p.x = tempArray[i].x + linePieceSize;
      p.y = tempArray[i].q + linePieceSize * tempArray[i].m;
      line_list.points.push_back(p);
    }

    linePub.publish(line_list);

    // Create the vertices for the points and lines
//    for (uint32_t i = 0; i < 100; ++i)
//    {
//      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

//      geometry_msgs::Point p;
//      p.x = (int32_t)i - 50;
//      p.y = y;
//      p.z = z;

//      points.points.push_back(p);
//      line_strip.points.push_back(p);

//      // The line list needs two points for each line
//      line_list.points.push_back(p);
//      p.z += 1.0;
//      line_list.points.push_back(p);
//    }

//    linePub.publish(line_list);

  }
  else{
    //@ToDoSave into file
  }



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
  pub = n.advertise<sensor_msgs::PointCloud2> ("output_from_my_filter", 1);
  linePub = n.advertise<visualization_msgs::Marker>("filteredRowinLines", 1);

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

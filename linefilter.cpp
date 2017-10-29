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
//#include "linerow.h"



typedef struct line{
  int x;
  float q;
  float m;
  float r;
  int value;
}line_t;

typedef line_t* line_p;

typedef struct lineRow{
  line_p LineRow;
  ros::Time timestamp;
  float velocity;
  int numberOfLines;
}lineRow_t;

typedef lineRow_t* lineRow_p;
    

#define ISSIMUALTION 1


lineRow_p lineRowArray;
float currentSpeed = 5.0;        //Current speed of the Drone needs to be updated.
float linePieceSize = 1.0;      //Size for one piece = 1m
float maxDifference = 1.5;      //Max Difference which is accepted bevor a staff etc. is detected.
int densityPerM = 20;           //min points per 1 meter to be accepted into the rating
int minValue = 50;              //min value which each line needs to have to be considered as good
int sizeOfRowArray = 1000;      //size of the Row Array equals the amount of Rows which are looked back to finde landing planes

int numberOfLines;


ros::Publisher pub;           //Publisher for the lineRow_p to safe it for later calculation of landing planes
ros::Publisher linePub;       //Publisher for the lineCloumn to visualize it in rviz


void PCColumnHandler(const sensor_msgs::PointCloud2ConstPtr& input){

  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PointCloud<pcl::PointXYZ> unfiltered_cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  cloud_filtered.header.frame_id ="VUX-1";
  pcl::PointCloud<pcl::PointXYZ> cloud_filteredConditional;

  // Covert to PCL data type
  pcl::fromROSMsg(*input, *cloud_unfiltered);

  // Perform the filtering
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (unfiltered_cloud);
//  sor.setLeafSize (0.1, 0.1,0);
//  sor.setMinimumPointsNumberPerVoxel (200);
//  sor.filter (cloud_filtered);
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
  filter.setMinNeighborsInRadius(3);
  filter.setRadiusSearch(0.5);
  filter.setInputCloud(cloud_unfiltered);
  filter.filter(cloud_filtered);

//  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>);
//  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, -10.0)));

  //Calculate the size of the lineArray which will be computet depending on x position of first and last entry of the given PC.

  numberOfLines = (int) (abs(cloud_filtered[0].x) + abs(cloud_filtered[cloud_filtered.width-1].x))/linePieceSize;

  line_p lineArray (new line_t[numberOfLines]);

  //Divide in defined pc pieces and convert them into line pieces

  int i = 0;
  int lineArrayIterator = 0;
  while(!(((int)cloud_filtered[i].x)%1 < 0.1 && ((int)cloud_filtered[i].x)%1 > -0.1)){ // Determinate the start point +- 1cm
    i ++;
  }

  int pos = (int) cloud_filtered[i].x;

  do{
    int start = i;

    float xstart = pos; //is needed later for calculation of the m value of the line


  while((cloud_filtered[i].x < (pos + linePieceSize)) && (i < (cloud_filtered.width-1))){
    i ++;
  }
    int end = i -1;


    int x = pos;
    pos = pos + (int) linePieceSize;

    int value = 0;
    double q;
    double m;
    float r;
    int size = abs(end - start)+1;

    if(size < (densityPerM/(int)linePieceSize)){ // if the densitiy is too small
      value = -1;
      q = 0.0;
      m = 0.0;
      r = 0.0;
    }
    else{

      //calculating the m and q value with the least square methode
      double xMeanNum = 0.0;
      double yMeanNum = 0.0;

      for(int i = start; i <= end;i++){
        xMeanNum += cloud_filtered[i].x - xstart; //this way everey x value of the array is treated as the line woud beginn at x = 0
        yMeanNum += cloud_filtered[i].y;
      }

      double xMean = xMeanNum/((float)size);
      double yMean = yMeanNum/((float)size);

      double numerator = 0.0;
      double denumerator = 0.0;

      for(int i = start; i <= end; i++){
        numerator += (cloud_filtered[i].x -xstart - xMean)*(cloud_filtered[i].y - yMean);
        denumerator += (cloud_filtered[i].x -xstart - xMean)*(cloud_filtered[i].x -xstart - xMean);
      }

      m = numerator/denumerator;


      q = yMean - (m*xMean);


      int counter = 0;
      r = 0.0;
      for(int i = start ; i <= end; i++){
        float onePointError = abs((m * (cloud_filtered[i].x -xstart) + q - cloud_filtered[i].y));

        if(onePointError > maxDifference){  //Test if the Error is greater then the maximal axepted Difference
          counter ++;
        }
        r += onePointError;
      }

      if(counter > 2){ //If there are too much height difference in more then one point -> probebly a staff or something
        value = -1;
      }
      else{
        //@ToDO value funktion z bsp.
        value = 1000 - (r*100 + abs((int)(m*1500))); //So it dosn't become a negative value if m is negatve. M is in percent.
      }

    }
    //just for debugging purpouse
    std::cout << "x: " <<x<<" m: "<< m <<" q: "<<q <<" r: " <<r << " value: " << value <<endl;
    std::cout <<"xstart:" << xstart <<" size:" << size <<" numberOfLines:" <<numberOfLines<< " lineArrayIterator: "<<lineArrayIterator <<endl;


    //Save calculated values int the Array
    lineArray[lineArrayIterator].x = x;
    lineArray[lineArrayIterator].q = q;
    lineArray[lineArrayIterator].m = m;
    lineArray[lineArrayIterator].r = r;
    lineArray[lineArrayIterator].value = value;

    lineArrayIterator ++;
  }
  while(lineArrayIterator < numberOfLines);


  //determinate if this is a Simulation -> publish lines, else store the made line array into he lineRow array.


  if(ISSIMUALTION){

    //@ToDo Convert into line pieces and publish it. Works more or less fine

    visualization_msgs::Marker line_list_bad;
    visualization_msgs::Marker line_list_good;
    visualization_msgs::Marker line_list_possible;
    line_list_bad.type = line_list_good.type = line_list_possible.type = visualization_msgs::Marker::LINE_LIST;

    line_list_bad.header.frame_id = line_list_possible.header.frame_id =  line_list_good.header.frame_id = "VUX-1";
    line_list_bad.header.stamp = line_list_possible.header.stamp = line_list_good.header.stamp = ros::Time::now();
    line_list_bad.ns = line_list_possible.ns =line_list_good.ns = "points_and_lines";
    line_list_bad.action = line_list_possible.action =line_list_good.action =visualization_msgs::Marker::ADD;
    line_list_bad.pose.orientation.w = line_list_possible.pose.orientation.w =  line_list_good.pose.orientation.w = 1.0;
    line_list_bad.id = 0;
    line_list_possible.id = 1;
    line_list_good.id = 2;

    // Set width of lines
    line_list_bad.scale.x = 0.15;
    line_list_possible.scale.x = 0.15;
    line_list_good.scale.x = 0.15;

    // Line list is red
    line_list_bad.color.r = 1.0;
    line_list_bad.color.a = 1.0;

    // Make possible orange
    line_list_possible.color.r = 0.5;
    line_list_possible.color.g = 0.5;
    line_list_possible.color.a = 1.0;

    // Make good Lines green
    line_list_good.color.g = 1.0f;
    line_list_good.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = 0;
    //determite if the lines are considered good possible or bad

    for(int i = 0; i < numberOfLines; i++){

    if(lineArray[i].value == -1 || lineArray[i].value < 500){
      p.x = lineArray[i].x;
      p.y = lineArray[i].q;
      line_list_bad.points.push_back(p);

      p.x = lineArray[i].x + linePieceSize;
      p.y = lineArray[i].q + lineArray[i].m * linePieceSize;
      line_list_bad.points.push_back(p);
    }
    else if(lineArray[i].value >= 700){
      p.x = lineArray[i].x;
      p.y = lineArray[i].q;
      line_list_good.points.push_back(p);

      p.x = lineArray[i].x + linePieceSize;
      p.y = lineArray[i].q + lineArray[i].m * linePieceSize;
      line_list_good.points.push_back(p);
    }
    else{
      p.x = lineArray[i].x;
      p.y = lineArray[i].q;
      line_list_possible.points.push_back(p);

      p.x = lineArray[i].x + linePieceSize;
      p.y = lineArray[i].q + lineArray[i].m * linePieceSize;
      line_list_possible.points.push_back(p);
    }
    }

    linePub.publish(line_list_bad);
    linePub.publish(line_list_possible);
    linePub.publish(line_list_good);

  }

  else{
    
    //@ToDoSave into file
    lineRow_p calculatedRow = new lineRow_t;
    calculatedRow->LineRow = lineArray;
    calculatedRow->timestamp = ros::Time::now();        // maybe should done differently
    calculatedRow->velocity = currentSpeed;  // current velocity of the drone
    calculatedRow->numberOfLines = numberOfLines;
    //handleLines(calculatedRow);

  }
}

void handleLines(lineRow_p lineRow){
   static int lineRowArrayIndexerStart = 0;
   static int lineRowArrayIndexerEnd = 0;

   lineRowArray[lineRowArrayIndexerEnd] = *lineRow;

   float aproxSpeed = (lineRowArray[lineRowArrayIndexerEnd].velocity +  lineRowArray[lineRowArrayIndexerStart].velocity)/2;
   float distanceStartEnd = (lineRowArray[lineRowArrayIndexerEnd].timestamp.toSec() - lineRowArray[lineRowArrayIndexerStart].timestamp.toSec())*aproxSpeed;
   //If there are enough lines scaned do the computing to get emergency landing planes for example if there is a 10m long scan
   //if((lineRowArrayIndexerEnd - lineRowArrayIndexerStart) > sizeOfRowArray/2){

   if(distanceStartEnd > 10){ // If there are more then 10m of scanned plane
      float possibleLandingSides[20]; //Should be enough because there is abou 150m which is scanned each row
      int posLineCounter = 0;
      int i = 0;
      int posLandingSidesCounter = 0;
      do{
        if(lineRowArray[lineRowArrayIndexerStart].LineRow[i].value > 500){
          posLineCounter ++;
          if(posLineCounter > (10/linePieceSize)){
            possibleLandingSides[posLandingSidesCounter] = lineRowArray[lineRowArrayIndexerStart].LineRow[i].x;
            posLandingSidesCounter ++;
            while(lineRowArray[lineRowArrayIndexerStart].LineRow[i].value > 500) // This way a flat place counts as just one plane not multible ones
            {
              i++;
            }
          }
        }
        else{
          posLineCounter = 0;
        }
        i++;
      }
      while(i < lineRowArray[lineRowArrayIndexerStart].numberOfLines);



   }









   lineRowArrayIndexerEnd ++;
   if(lineRowArrayIndexerEnd == sizeOfRowArray){
     lineRowArrayIndexerEnd = 0;
   }
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

  lineRowArray = new lineRow_t[sizeOfRowArray];
  
  
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

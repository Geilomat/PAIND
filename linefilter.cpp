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

typedef struct possibleLandingSides{
  int value;
  int time;
  int xPos;
  float velocity;
}possibleLandingSides_t;

typedef possibleLandingSides_t* possibleLandingSides_p;
    

#define IS_SIMULATION 3          //define if this is a simulation or runnning on real time
#define NUMBER_OF_POSSIBLE_LANDING_SIDES 10 //# of possible landing sides which shoulde be stored

possibleLandingSides_p posLandingSideArray; //Array where the possible landing sides are stored;
lineRow_p* lineRowArray;         //Array of the scanned lines is handled as ringbuffer.
float currentSpeed = 5.0;       //Current speed of the Drone needs to be updated.
float linePieceSize = 1.0;      //Size for one piece = 1m
float maxDifference = 0.5;      //Max Difference which is accepted bevor a staff etc. is detected.
int densityPerM = 10;           //min points per 1 meter to be accepted into the rating
int minValue = 500;              //min value which each line needs to have to be considered as good
int sizeOfRowArray = 1000;      //size of the Row Array equals the amount of Rows which are looked back to finde landing planes

int numberOfLines;


ros::Publisher pub;           //Publisher for the lineRow_p to safe it for later calculation of landing planes
ros::Publisher linePub;       //Publisher for the lineCloumn to visualize it in rviz



void handleLines(lineRow_p lineRow){
   static int lineRowArrayIndexerStart = 0;
   static int lineRowArrayIndexerEnd = 0;

   lineRowArray[lineRowArrayIndexerEnd] = lineRow;

   float aproxSpeed = (lineRowArray[lineRowArrayIndexerEnd]->velocity +  lineRowArray[lineRowArrayIndexerStart]->velocity)/2;
   float distanceStartEnd = (lineRowArray[lineRowArrayIndexerEnd]->timestamp.toSec() - lineRowArray[lineRowArrayIndexerStart]->timestamp.toSec())*aproxSpeed;
   //If there are enough lines scaned do the computing to get emergency landing planes for example if there is a 10m long scan
   //if((lineRowArrayIndexerEnd - lineRowArrayIndexerStart) > sizeOfRowArray/2){

   if(distanceStartEnd > 10){ // If there are more then 10m of scanned plane

#if(IS_SIMULATION == 3)

     visualization_msgs::Marker line_list_good;
     line_list_good.type = visualization_msgs::Marker::LINE_LIST;
     line_list_good.lifetime = ros::Duration(2,0);
     line_list_good.header.frame_id = "VUX-1";
     line_list_good.header.stamp = ros::Time::now();
     line_list_good.ns = "points_and_lines";
     line_list_good.action =visualization_msgs::Marker::ADD;
     line_list_good.pose.orientation.w = 1.0;
     line_list_good.id = 0;

     // Set width of lines
     line_list_good.scale.x = 0.5;


     // Make good Lines green
     line_list_good.color.g = 1.0f;
     line_list_good.color.a = 1.0;

#endif
     //Array for the possible landing sides first entry is x value second is the index of the first lineRow = lineRowArrayIndexerStart
      int possibleLandingSides[20][2]; //Should be enough because there is about 150m which is scanned each row
      int posLineCounter = 0;
      int i = 0;
      int posLandingSidesCounter = 0;
      do{
        // Calculate the difference in hight bewtween the different lines.
        float qDifference = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].q - lineRowArray[lineRowArrayIndexerStart]->LineRow[i+ 1].q);
        if(i > 1){
          qDifference += abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i-1].q - lineRowArray[lineRowArrayIndexerStart]->LineRow[i].q);
        }



        if(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > minValue && qDifference < 1.0){
          posLineCounter ++;
          if(posLineCounter > (10/linePieceSize)){
            possibleLandingSides[posLandingSidesCounter][0] = lineRowArray[lineRowArrayIndexerStart]->LineRow[i].x;
            possibleLandingSides[posLandingSidesCounter][1] = i;
#if (IS_SIMULATION == 2)

            std::cout << "possibleLandingsideDetected at: " << possibleLandingSides[posLandingSidesCounter][0] << endl;
#endif
            posLandingSidesCounter ++;
            while(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > minValue) // This way a flat place counts as just one plane not multible ones
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
      while(i < lineRowArray[lineRowArrayIndexerStart]->numberOfLines);
#if (IS_SIMULATION != 1)
      // Check if there are possible landingsides which can be computed
      if(posLandingSidesCounter > 0){
      //@ToDo: find here the possible landing sites and give them a value;
        i = 0;
        do{
          int indexer = 0;
          int upLinesChecker = 0;
          int goodColumnsCounter = 0;
          while(lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].value > 500){

            //find the same line in the upper rows an check there values
            upLinesChecker = 1;
            for(int y = lineRowArrayIndexerStart + 1; y <= lineRowArrayIndexerEnd ; y++) //this loop iterates from the give start line upwards in lineRows of the lineRowArray
            {
              int j= 0;
              while(lineRowArray[y]->LineRow[j].x != possibleLandingSides[i][0]) j++;  // find the line in the upper lineRow with the same x position as the one at the bottom of the Array;
                  if(lineRowArray[y]->LineRow[j].value < minValue) upLinesChecker = 0;      // if the value from just one line is considered as bad.

            }
            if(upLinesChecker){ // if all upper lines were also good.
              goodColumnsCounter ++;
            }
            else goodColumnsCounter = 0;

            if(goodColumnsCounter >= (10/linePieceSize)){ //10 good Columns where detectet -> ah possible landing side
              // publish here a possible landing field with the array of the lines -> value, time, speed etc.
#if (IS_SIMULATION ==3)

              std::cout <<"possible landingside detectet. Lower left corner at: " <<  lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x -10 << endl;
              geometry_msgs::Point p;
              p.z = 0;

              p.x = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x -10;
              p.y = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].q;
              line_list_good.points.push_back(p);

              p.x = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x;
              line_list_good.points.push_back(p);



#endif
              goodColumnsCounter = 0;
            }


            indexer++;
          }
          i++;
        }
        while(i < posLandingSidesCounter);
      }
#endif

#if (IS_SIMULATION == 3)
      linePub.publish(line_list_good);

#endif


      //delete the oldest few entries
      int currentEntries;
#if (IS_SIMULATION == 2)

            std::cout << "trying to delet the older Entries" << endl;
#endif
#if (IS_SIMULATION != 1)
      if(lineRowArrayIndexerStart > lineRowArrayIndexerEnd){
        currentEntries = sizeOfRowArray - lineRowArrayIndexerStart + lineRowArrayIndexerEnd;
      }
      else{
        currentEntries = lineRowArrayIndexerEnd - lineRowArrayIndexerStart;
      }
      i = 0;
      while(i <=currentEntries/2){ // Delet aprox halve of the entries
        delete lineRowArray[lineRowArrayIndexerStart + i];
        i ++;
        //lineRowArrayIndexerStart ++;
        if(lineRowArrayIndexerStart + i == sizeOfRowArray){
          lineRowArrayIndexerStart = 0;
          currentEntries - i*2;
          i = 0;
        }

      }
      lineRowArrayIndexerStart = lineRowArrayIndexerStart + i;
#endif

   }


   lineRowArrayIndexerEnd ++;
   if(lineRowArrayIndexerEnd == sizeOfRowArray){
     lineRowArrayIndexerEnd = 0;
   }
}


void PCRowHandler(const sensor_msgs::PointCloud2ConstPtr& input){

  // Create a container for the data.


  // Do data processing here...

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  cloud_filtered.header.frame_id ="VUX-1";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredConditional (new pcl::PointCloud<pcl::PointXYZ>);

  // Covert to PCL data type
  pcl::fromROSMsg(*input, *cloud_unfiltered);

  // Perform the filtering
//  pcl::VoxelGrid<pcl::PointXYZ> sor;
//  sor.setInputCloud (unfiltered_cloud);
//  sor.setLeafSize (0.1, 0.1,0);
//  sor.setMinimumPointsNumberPerVoxel (200);
//  sor.filter (cloud_filtered);
  pcl::PassThrough<pcl::PointXYZ> ptfilter (new pcl::PassThrough<pcl::PointXYZ>); // Initializing with true will allow us to extract the removed indices
  ptfilter.setInputCloud(cloud_unfiltered);
  ptfilter.setFilterFieldName ("y");
  ptfilter.setFilterLimits (-1000.0, -20.0);
  ptfilter.filter(*cloud_filteredConditional);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
  filter.setMinNeighborsInRadius(1);
  filter.setRadiusSearch(0.3);
  filter.setInputCloud(cloud_filteredConditional);
  filter.filter(cloud_filtered);



//  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>);
//  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, -5.0)));
//  range_cond.
  //Calculate the size of the lineArray which will be computet depending on x position of first and last entry of the given PC.

  numberOfLines = (int) (abs(cloud_filtered[0].x) + abs(cloud_filtered[cloud_filtered.width-1].x))/linePieceSize;


#if (IS_SIMULATION ==1)
  std::cout << "X start:" << cloud_filtered[0].x << " X end:" << cloud_filtered[cloud_filtered.width-1].x << endl;
  line_p lineArray (new line_t[numberOfLines]);

#else
  line_p lineArray = new line_t[numberOfLines];
#endif
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
          value = -1;  //If there are too much height difference in more then one point -> probebly a staff or something
        }
        r += onePointError;
      }

      if(value > -1){
        //@ToDO value funktion z bsp.
        value = 1000 - (r*100 + abs((int)(m*1500))); //So it dosn't become a negative value if m is negatve. M is in percent.
      }

    }

#if (IS_SIMULATION == 1)
    //just for debugging purpouse
    std::cout << "x: " <<x<<" m: "<< m <<" q: "<<q <<" r: " <<r << " value: " << value <<endl;
    std::cout <<"xstart:" << xstart <<" size:" << size <<" numberOfLines:" <<numberOfLines<< " lineArrayIterator: "<<lineArrayIterator <<endl;
#endif

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


#if (IS_SIMULATION == 1)

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
    line_list_possible.color.r = 1;
    line_list_possible.color.g = 0.55;
    line_list_possible.color.a = 1.0;

    // Make good Lines green
    line_list_good.color.g = 1.0f;
    line_list_good.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = 0;
    //determite if the lines are considered good possible or bad

    for(int i = 0; i < numberOfLines; i++){

    if(lineArray[i].value == -1 || lineArray[i].value < minValue){
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

    //Publish the filtered cloud vor visualization purpose
    sensor_msgs::PointCloud2 output;
    output.header.frame_id = "VUX-1";
    pcl::toROSMsg(cloud_filtered, output);

    pub.publish(output);
#else
    
    //@ToDoSave into file
    lineRow_p calculatedRow = new lineRow_t;
    calculatedRow->LineRow = lineArray;
    calculatedRow->timestamp = ros::Time::now();        // maybe should done differently
    calculatedRow->velocity = currentSpeed;  // current velocity of the drone
    calculatedRow->numberOfLines = numberOfLines;
    handleLines(calculatedRow);

#if (IS_SIMULATION == 3)
      sensor_msgs::PointCloud2 output;
      output.header.frame_id = "VUX-1";
      pcl::toROSMsg(cloud_filtered, output);
      //std::cout << "Points per row:" << cloud_unfiltered->width << endl;

      pub.publish(output);
#endif

#endif
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
  ros::Subscriber sub = n.subscribe("point_cloud_unfiltered", 1, PCRowHandler);
// %EndTag(SUBSCRIBER)%


  // Create a ROS publisher for the output point cloud
  pub = n.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_from_linefilter", 1);
  linePub = n.advertise<visualization_msgs::Marker>("filteredRowinLines", 1);

  lineRowArray = new lineRow_p[sizeOfRowArray];
  
  
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

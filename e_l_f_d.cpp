
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
#include "e_l_f_d.h"

//specific include for the columns
//#include "linerow.h"




//possibleLandingField_p* posLandingFieldArray; //Array where the possible landing sides are stored;
lineRow_p lineRowArray[SIZE_OF_ROW_BUFFER];         //Array of the scanned lines is handled as ringbuffer.
float currentSpeed = 5.0;       //Current speed of the Drone needs to be updated ! [m/s]
double startTime;

int32_t voltageLinePositionX = 0;


ros::Publisher pub;           //Publisher for the lineRow_p to safe it for later calculation of landing planes
ros::Publisher linePub;       //Publisher for the lineCloumn to visualize it in rviz
ros::Publisher possibleLandingFieldsPub; //publisher for possible landing fields


#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)
visualization_msgs::Marker line_list_bad;
visualization_msgs::Marker line_list_good;
visualization_msgs::Marker line_list_possible;
#endif

static void PCRowHandler(const sensor_msgs::PointCloud2ConstPtr& input){

  static double oldTime = startTime;
  ros::Time timestamp  = ros::Time::now(); //get the actual time;
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  cloud_filtered.header.frame_id ="VUX-1";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredConditional (new pcl::PointCloud<pcl::PointXYZ>);

  // Covert to PCL data type
  pcl::fromROSMsg(*input, *cloud_unfiltered);

  if(cloud_unfiltered->width > 0){

  // Perform the filtering
  pcl::PassThrough<pcl::PointXYZ> ptfilter (new pcl::PassThrough<pcl::PointXYZ>); // Initializing with true will allow us to extract the removed indices
  ptfilter.setInputCloud(cloud_unfiltered);
  ptfilter.setFilterFieldName ("y");
  ptfilter.setFilterLimits (-1000.0, -20);
  ptfilter.filter(*cloud_filteredConditional);

  if(cloud_filteredConditional->width >0){

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
  filter.setMinNeighborsInRadius(1);
  filter.setRadiusSearch(0.3);
  filter.setInputCloud(cloud_filteredConditional);
  filter.filter(cloud_filtered);

  if(cloud_filtered.width > 250){
  //Calculate the size of the lineArray which will be computet depending on x position of first and last entry of the given PC.
  int numberOfLines = (int) (abs(cloud_filtered[0].x) + abs(cloud_filtered[cloud_filtered.width-1].x))/LINE_PIECE_SIZE;


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

#if (IS_SIMULATION == 7)
    //std::cout <<"a" <<endl;
#endif

  do{
    int start = i;

    float xstart = pos; //is needed later for calculation of the m value of the line


  while((cloud_filtered[i].x < (pos + LINE_PIECE_SIZE)) && (i < (cloud_filtered.width-1))){
    i ++;
  }
    int end = i -1;


    int x = pos;
    pos = pos + (int) LINE_PIECE_SIZE;

    int value = 0;
    double yStart;
    double slope;
    float roughness;
    int size = abs(end - start)+1;

    if(voltageLinePositionX == 0){
     // std::cout << "DANGER \t !!! line tracking isn't working !!! \n It is possible that landing fields will be detected under the voltage line !!!" << endl;
    }
    else{   //check if the new line is under the voltage line -> make size = 0 => line will considerer as bad because of density
      if((xstart > voltageLinePositionX-VOLTAGE_LINE_SIZE/2 && xstart < voltageLinePositionX+VOLTAGE_LINE_SIZE/2) ||
         (pos > voltageLinePositionX-VOLTAGE_LINE_SIZE/2 && pos < voltageLinePositionX+VOLTAGE_LINE_SIZE/2)){
        size = 0;
      }

    }


    if(size < (DENSITY_PER_M/LINE_PIECE_SIZE)){ // if the densitiy is too small
      value = -1;
      yStart = 0.0;
      slope = 0.0;
      roughness = 0.0;
    }
    else{
#if (IS_SIMULATION == 7)
    //std::cout <<"b" <<endl;
#endif

      //calculating the slope and yStart value with the least square methode
      double xMean = 0.0;
      double yMean = 0.0;
      double xSquared = 0.0;
      double xyMul = 0.0;

      for(int i = start; i<=end; i++){
        xMean += cloud_filtered[i].x - xstart; //this way every x value of the array is treated as the line woud begin at x = 0
        yMean += cloud_filtered[i].y;
        xSquared += (cloud_filtered[i].x-xstart)*(cloud_filtered[i].x -xstart);
        xyMul += (cloud_filtered[i].x - xstart) * cloud_filtered[i].y;
      }

      xMean = xMean/((float)size);
      yMean = yMean/((float)size);


      slope = (xyMul - (size*xMean*yMean))/(xSquared - (size*xMean*xMean));
      yStart = yMean - (slope*xMean);

      //calculate the average roughness
      roughness = 0.0;
      for(int i = start ; i <= end; i++){
        float onePointError = abs((slope * (cloud_filtered[i].x -xstart) + yStart - cloud_filtered[i].y));

        if(onePointError > MAX_ACCEPTED_DIFFERENCE){  //Test if the Error is greater then the maximal axepted Difference
          value = -1;  //If there are too much height difference in more then one point -> probebly a staff or something
        }
        roughness += onePointError;
      }
      roughness = roughness/size;

      if(value > -1){
        //@ToDO value funktion z bsp.
        value = MAX_VALUE_LINE - (roughness*100 + abs(slope*((MAX_VALUE_LINE-MIN_VALUE)/MAX_ACCEPTED_SLOPE))); //So it dosn't become a negative value if slope is negatve. Slope is in percent.
      }

    }

#if (IS_SIMULATION == 1)
    //just for debugging purpouse
    std::cout << "x: " <<x<<" slope: "<< slope<<" ystart: "<<yStart <<" roughness: " <<roughness << " value: " << value <<endl;
    std::cout <<"xstart:" << xstart <<" size:" << size <<" numberOfLines:" <<numberOfLines<< " lineArrayIterator: "<<lineArrayIterator <<endl;
#endif

    //Save calculated values into the Array
    lineArray[lineArrayIterator].x = x;
    lineArray[lineArrayIterator].yStart = yStart;
    lineArray[lineArrayIterator].slope = slope;
    lineArray[lineArrayIterator].roughness = roughness;
    lineArray[lineArrayIterator].value = value;

    lineArrayIterator ++;
  }
  while(lineArrayIterator < numberOfLines);
#if (IS_SIMULATION == 7)
   // std::cout <<"c" <<endl;
#endif

  //determinate if this is a Simulation -> publish lines, else store the made line array into he lineRow array.
double z = (timestamp.toSec() - startTime) * currentSpeed;

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)

    //Convert into line pieces and publish it.

  if((timestamp.toSec() - oldTime) * currentSpeed > 0.5){
    oldTime = timestamp.toSec();

    // Set width of lines
    line_list_bad.scale.x = 0.15;
    line_list_possible.scale.x = 0.15;
    line_list_good.scale.x = 0.15;

    // Line list is red
    line_list_bad.color.r = 1.0;
    line_list_bad.color.a = 1.0;

    // Make possible orange
    line_list_possible.color.r = 0.6;
    line_list_possible.color.g = 0.55;
    line_list_possible.color.a = 1.0;

    // Make good Lines green
    line_list_good.color.g = 1.0f;
    line_list_good.color.a = 1.0;

    geometry_msgs::Point p;

    p.z = z;

    //determite if the lines are considered good possible or bad

    for(int i = 0; i < numberOfLines; i++){
#if (IS_SIMULATION != 2)
    if(lineArray[i].value != -1 ){
#else
      {
#endif

    if(lineArray[i].value < MIN_VALUE){
      p.x = lineArray[i].x;
      p.y = lineArray[i].yStart;
      line_list_bad.points.push_back(p);

      p.x = lineArray[i].x + LINE_PIECE_SIZE;
      p.y = lineArray[i].yStart + lineArray[i].slope * LINE_PIECE_SIZE;
      line_list_bad.points.push_back(p);
    }
    else if(lineArray[i].value >= 700){
      p.x = lineArray[i].x;
      p.y = lineArray[i].yStart;
      line_list_good.points.push_back(p);

      p.x = lineArray[i].x + LINE_PIECE_SIZE;
      p.y = lineArray[i].yStart + lineArray[i].slope * LINE_PIECE_SIZE;
      line_list_good.points.push_back(p);
    }
    else{
      p.x = lineArray[i].x;
      p.y = lineArray[i].yStart;
      line_list_possible.points.push_back(p);

      p.x = lineArray[i].x + LINE_PIECE_SIZE;
      p.y = lineArray[i].yStart + lineArray[i].slope * LINE_PIECE_SIZE;
      line_list_possible.points.push_back(p);
    }
    }
  }
  }
    //Publish the filtered cloud vor visualization purpose
    sensor_msgs::PointCloud2 output;
    output.header.frame_id = "VUX-1";
    pcl::toROSMsg(cloud_filtered, output);

    pub.publish(output);
#if (IS_SIMULATION == 1)
    return;
#endif
#endif

#if (IS_SIMULATION == 7)
    std::cout <<"d" <<endl;
#endif

    lineRow_p calculatedRow = new lineRow_t;
    calculatedRow->LineRow = lineArray;
    calculatedRow->timestamp = timestamp;               // maybe should done differently
    calculatedRow->velocity = currentSpeed;             // current velocity of the drone
    calculatedRow->numberOfLines = numberOfLines;
    calculatedRow->z = z;
    handleLines(calculatedRow);

#if (IS_SIMULATION == 3)
      sensor_msgs::PointCloud2 output;
      output.header.frame_id = "VUX-1";
      pcl::toROSMsg(cloud_filtered, output);
      //std::cout << "Points per row:" << cloud_unfiltered->width << endl;

      pub.publish(output);
#endif
  }
  else{
    std::cout << "Filtered Cloud is to small for computing !!" << endl;
  }
  }
  else{
    std::cout << "Sensor is too close to the ground. Can't scan for landingfields !!" << endl;
  }
  }
}



static void handleLines(lineRow_p lineRow){
   static int lineRowArrayIndexerStart = 0;
   static int lineRowArrayIndexerEnd = 0;
   static int currentEntries = 0;

   lineRowArray[lineRowArrayIndexerEnd] = lineRow;
   currentEntries ++;

#if (IS_SIMULATION == 7)
    std::cout <<"e" <<endl;
#endif

   float aproxSpeed = (lineRowArray[lineRowArrayIndexerEnd]->velocity +  lineRowArray[lineRowArrayIndexerStart]->velocity)/2;
   float distanceStartEnd = (lineRowArray[lineRowArrayIndexerEnd]->timestamp.toSec() - lineRowArray[lineRowArrayIndexerStart]->timestamp.toSec())*aproxSpeed;
#if (IS_SIMULATION == 2 || IS_SIMULATION == 4 || IS_SIMULATION == 7)

            std::cout << "distanceStartEnd: " << distanceStartEnd << endl;
#endif

   if(distanceStartEnd > LANDING_FIELD_SIZE){ // If there are more then the hight of a landing field of scanned plane

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
     int maxPossLandingField = (lineRowArray[lineRowArrayIndexerStart]->numberOfLines * LINE_PIECE_SIZE)/LANDING_FIELD_SIZE;
     int possibleLandingSides[maxPossLandingField][2]; //Should be enough because there is about 150m which is scanned each row
     int posLineCounter = 0;
     int i = 0;
     int posLandingSidesCounter = 0;
#if (IS_SIMULATION == 7)
    std::cout <<"e.1" <<endl;
#endif
      do{
        // Calculate the difference in hight bewtween the different lines.
        float qDifference = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].yStart - lineRowArray[lineRowArrayIndexerStart]->LineRow[i+ 1].yStart);
        if(i > 1){
          qDifference += abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i-1].yStart - lineRowArray[lineRowArrayIndexerStart]->LineRow[i].yStart);
        }

        if(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > MIN_VALUE && qDifference < MAX_ACCEPTED_DIFFERENCE){
          posLineCounter ++;
          if(posLineCounter > (LANDING_FIELD_SIZE/LINE_PIECE_SIZE)){
            possibleLandingSides[posLandingSidesCounter][0] = lineRowArray[lineRowArrayIndexerStart]->LineRow[i].x;
            possibleLandingSides[posLandingSidesCounter][1] = i;
#if (IS_SIMULATION == 2 || IS_SIMULATION == 4)
            std::cout << "posLandingSidesCounter: " << posLandingSidesCounter<<  endl;
#endif
#if (IS_SIMULATION == 2)

            std::cout << "possibleLandingsideDetected at: " << possibleLandingSides[posLandingSidesCounter][0] << endl;
#endif
            posLandingSidesCounter ++;
            while(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > MIN_VALUE) // This way a flat place counts as just one plane not multible ones
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
#if (IS_SIMULATION == 7)
    std::cout <<"f" <<endl;
#endif
#if (IS_SIMULATION != 1)
      // Check if there are possible landingsides which can be computed
      if(posLandingSidesCounter > 0){
        i = 0;
        do{
          int indexer = 0;
          int upLinesChecker = 0;
          int goodColumnsCounter = 0;
          int landingFieldSize = 0;
          long landingFieldValue = 0;
          int startingPoints[currentEntries];
          int goodColumns = 1;
          float slope = 0;
          //int64_t xCenterPos = 0xFFFFFFFFFFFFFFFF; //this way it can be safely detected if the variable was writen or not. (0xFFFFFFFFFFFFFFFF should never occur in a normal running programm)
          while(goodColumns){
            if(lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].value < MIN_VALUE){
              goodColumns = 0;
            }
            //find the same line in the upper rows an check there values
              upLinesChecker = 1;
              //std::cout << "1"<<  endl;
              int y = lineRowArrayIndexerStart + 1;
              int z = 0;
              while(y != lineRowArrayIndexerEnd && (upLinesChecker || !indexer)){ //this loop iterates from the given start line upwards in lineRows
                if(y >= SIZE_OF_ROW_BUFFER) {y = 0;}
              int j = 0;
              //std::cout << "2"<<  endl;
              if(indexer == 0){
                while(lineRowArray[y]->LineRow[j].x != possibleLandingSides[i][0] && upLinesChecker){  //find the line in the upper lineRow with the same x position as the one at the bottom of the Array
                  j++;
                  if(j >= (lineRowArray[y]->numberOfLines -1)){upLinesChecker = 0;}
                }
                startingPoints[z] = j;
                if(upLinesChecker){
                //  std::cout << "3"<<  endl;
                  if(lineRowArray[y]->LineRow[j].value < MIN_VALUE) {upLinesChecker = 0;}
                  else{
                    landingFieldSize ++;
                    landingFieldValue += lineRowArray[y]->LineRow[j].value;
                  }
                }

              }
              else{
                if(startingPoints[z]+indexer < lineRowArray[y]->numberOfLines){
                  if(lineRowArray[y]->LineRow[startingPoints[z]+indexer].value < MIN_VALUE) {upLinesChecker = 0;} // if the value from just one line is considered as bad.
                  else{
                    landingFieldSize ++;
                    landingFieldValue += lineRowArray[y]->LineRow[startingPoints[z]+indexer].value;
                  }
                }
                else{
                  //std::cout << "5"<<  endl;
                  upLinesChecker = 0;
                }
              }
              //std::cout << "6"<<  endl;
              y ++;
              z ++;

#if (IS_SIMULATION == 2 || IS_SIMULATION == 3)
            std::cout << "y: " << y<<  "\t j: " << j<<  endl;
            std::cout <<     " z: " << z <<
                             "\t y: " << y <<
                             "\t startingPoint: " << startingPoints[z] <<
                             "\t indexer: " << indexer <<
                             "\t numOfLi: " << lineRowArray[y]->numberOfLines << endl;

#endif
              }
            if(upLinesChecker){ // if all upper lines were also good.
              slope = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].yStart - lineRowArray[lineRowArrayIndexerEnd]->LineRow[startingPoints[z-1]+indexer].yStart)/LANDING_FIELD_SIZE;
               //std::cout << "slope: "<< slope << endl;
              if(slope < MAX_ACCEPTED_SLOPE){ //check if slope in z-axis is also good.
                goodColumnsCounter ++;
              }
            }
            else if(goodColumnsCounter < LANDING_FIELD_SIZE/LINE_PIECE_SIZE){
              landingFieldSize = 0;
              landingFieldValue  = 0;
              goodColumnsCounter = 0;
              //std::cout<<"was here"<< endl;
            }

//            if(goodColumnsCounter >= (LANDING_FIELD_SIZE/LINE_PIECE_SIZE)){
//              std::cout << goodColumnsCounter/LINE_PIECE_SIZE << endl;
//            }


            if(goodColumnsCounter >= (LANDING_FIELD_SIZE/LINE_PIECE_SIZE) && !upLinesChecker){ //Enough good Columns where detectet -> ah possible landing field
              //publish here a possible landing field with the array of the lines -> value, time, speed etc.

              //check first if slope on the sides is okay:
              //float leftSideSlope = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer-goodColumnsCounter].yStart - lineRowArray[lineRowArrayIndexerEnd]->LineRow[possibleLandingSides[i][1]+indexer].yStart)/LANDING_FIELD_SIZE;
              //float rightSideSlope = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer-goodColumnsCounter].yStart - lineRowArray[lineRowArrayIndexerEnd]->LineRow[possibleLandingSides[i][1]+indexer].yStart)/LANDING_FIELD_SIZE;

              //if(leftSideSlope <= MAX_ACCEPTED_SLOPE && rightSideSlope <= MAX_ACCEPTED_SLOPE){

                //make a container for the LandingField and fill it with the right values:
              pcl_filter::LandingField newPossLandField;

              std::cout << "tada " << goodColumnsCounter/LINE_PIECE_SIZE << endl;
              newPossLandField.value = (landingFieldValue/landingFieldSize) - (slope * 100) + (goodColumnsCounter - LANDING_FIELD_SIZE/LINE_PIECE_SIZE)*10;
              newPossLandField.length = goodColumnsCounter/LINE_PIECE_SIZE;
              newPossLandField.xPos = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x -  (newPossLandField.length/2);
              int middleLine = lineRowArrayIndexerStart + currentEntries/2;
              if(middleLine >= SIZE_OF_ROW_BUFFER-1){
                middleLine = currentEntries/2 - (SIZE_OF_ROW_BUFFER-1-lineRowArrayIndexerStart);
              }
              newPossLandField.timestamp = lineRowArray[middleLine]->timestamp;
              newPossLandField.velocity = lineRowArray[middleLine]->velocity;
              newPossLandField.z = lineRowArray[middleLine]->z;
              newPossLandField.hight = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+(int)(goodColumnsCounter/2)].yStart;
              newPossLandField.width = LANDING_FIELD_SIZE;

#if (IS_SIMULATION == 4 || IS_SIMULATION == 6)
              std::cout << "currentEntries: " << currentEntries << endl;
              std::cout << "lineRowArrayIndexerStart: " << lineRowArrayIndexerStart  << endl;
              std::cout << "lineRowArrayIndexerEnd: " << lineRowArrayIndexerEnd  << endl;
              std::cout << "middleLine: " << middleLine <<"\t z: " << lineRowArray[middleLine]->z << endl;
              std::cout << "Landingfield:  value: " << newPossLandField.value <<
                             "\t hight: " << newPossLandField.hight<<
                             "\t slope: " << slope <<
                             "\t GValue: "  <<landingFieldValue <<
                             "\t size: "  << landingFieldSize<<
                             "\t z: " << newPossLandField.z <<
                             "\t xPos: " << newPossLandField.xPos <<
                             "\t length " << newPossLandField.length << endl;

#endif


                //publish it so it can be stored into the Array;
                possibleLandingFieldsPub.publish(newPossLandField);

#if (IS_SIMULATION ==3)

                std::cout <<"possible landingside detectet. Lower left corner at: " <<  lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x -LANDING_FIELD_SIZE << endl;
                geometry_msgs::Point p;
                p.z = 0;

                p.x = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x -LANDING_FIELD_SIZE;
                p.y = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].yStart;
                line_list_good.points.push_back(p);

                p.x = lineRowArray[lineRowArrayIndexerStart]->LineRow[possibleLandingSides[i][1]+indexer].x;
                line_list_good.points.push_back(p);
#endif
              //}
              landingFieldSize = 0;
              landingFieldValue  = 0;
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

#if (IS_SIMULATION == 2 || IS_SIMULATION == 4)

            std::cout << "trying to delet the older Entries" << endl;
            std::cout << "currentEntries: " << currentEntries << endl;
            std::cout << "lineRowArrayIndexerStart: " << lineRowArrayIndexerStart  << endl;
            std::cout << "lineRowArrayIndexerEnd: " << lineRowArrayIndexerEnd  << endl;

#endif
#if (IS_SIMULATION != 1)

      i = 0;
      int currentEntriesTemp = currentEntries;
      while(i <=currentEntriesTemp/2){ // Delete aprox halve of the entries

        delete [] lineRowArray[lineRowArrayIndexerStart + i]->LineRow;
        delete lineRowArray[lineRowArrayIndexerStart + i];
        lineRowArray[lineRowArrayIndexerStart] = NULL;
        i ++;
        if(lineRowArrayIndexerStart + i == SIZE_OF_ROW_BUFFER){
          lineRowArrayIndexerStart = 0;
          currentEntriesTemp = currentEntriesTemp - (i*2);
          currentEntries = currentEntries -i;
          i = 0;
        }

      }
      currentEntries = currentEntries -i;
      lineRowArrayIndexerStart = lineRowArrayIndexerStart + i;
#endif

   }


   lineRowArrayIndexerEnd ++;
   if(lineRowArrayIndexerEnd == SIZE_OF_ROW_BUFFER){
     lineRowArrayIndexerEnd = 0;
   }
#if (IS_SIMULATION == 2 || IS_SIMULATION == 4 || IS_SIMULATION == 7)

            std::cout << "loop finished" << endl;
#endif
}

static void updateVoltageLinePos(std_msgs::Int32 xDistance){
  voltageLinePositionX = xDistance.data/100;
}

static void updateVelocity(std_msgs::Int32 newVelocity){
  currentSpeed = newVelocity.data;
}

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6 || IS_SIMULATION == 2)
static void publishLineList(const ros::TimerEvent& event){

  linePub.publish(line_list_bad);
  linePub.publish(line_list_possible);
  linePub.publish(line_list_good);


}
#endif

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
  ros::init(argc, argv, "e_l_f_d");

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
  ros::Subscriber sub2 = n.subscribe("line_distance",1,updateVoltageLinePos);
  //ros::Subscriber sub3 = n.subscribe("UAV_velocity",1,updateVelocity);
// %EndTag(SUBSCRIBER)%


  // Create a ROS publisher for the output point cloud
  pub = n.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_from_linefilter", 1);
  linePub = n.advertise<visualization_msgs::Marker>("filteredRowinLines", 1);
  possibleLandingFieldsPub = n.advertise<pcl_filter::LandingField>("possLandingFields",1);


  startTime = ros::Time::now().toSec();

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)
  ros::Timer timer = n.createTimer(ros::Duration(2),publishLineList,false);




  line_list_bad.type = line_list_good.type = line_list_possible.type = visualization_msgs::Marker::LINE_LIST;

  line_list_bad.header.frame_id = line_list_possible.header.frame_id =  line_list_good.header.frame_id = "VUX-1";
  line_list_bad.header.stamp = line_list_possible.header.stamp = line_list_good.header.stamp = ros::Time::now();
  line_list_bad.ns = line_list_possible.ns =line_list_good.ns = "points_and_lines";
  line_list_bad.action = line_list_possible.action =line_list_good.action =visualization_msgs::Marker::ADD;
  line_list_bad.pose.orientation.w = line_list_possible.pose.orientation.w =  line_list_good.pose.orientation.w = 1.0;
  line_list_bad.id = 0;
  line_list_possible.id = 1;
  line_list_good.id = 2;

#endif
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

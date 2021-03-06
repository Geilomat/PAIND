
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
#include "e_l_f_d.h"    //contents the configuration defines and different used typedefs


//possibleLandingField_p* posLandingFieldArray; //Array where the possible landing sides are stored;
lineRow_p lineRowArray[SIZE_OF_ROW_BUFFER];         //Array of the scanned lines is handled as ringbuffer.
float currentSpeed = 0.0;       //Current speed of the Drone needs to be updated ! [m/s]
float headingAngle = 0;         //Heading angle to detect turn manouvers [rad]
int turnCounter = 0;
int deleteEntriesFlag = 0;
double oldTime;                  //Starting time of the algorithm to find the z position of the landing field corresponding to the starting point.

#if IS_SIMULATION
double oldTimeForVis;
#endif

double zValue = 0;
ros::Time Timer;

int32_t voltageLinePositionX = 0;
ros::Publisher possibleLandingFieldsPub; //publisher for possible landing fields


#if IS_SIMULATION
ros::Publisher pub;             //Publisher for the lineRow_p to safe it for later calculation of landing planes
ros::Publisher linePub;         //Publisher for the lineCloumn to visualize it in rviz
#endif

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)

visualization_msgs::Marker line_list_bad;
visualization_msgs::Marker line_list_good;
visualization_msgs::Marker line_list_possible;

#endif

static void PCRowHandler(const sensor_msgs::PointCloud2ConstPtr& input){


  ros::Time timestamp  = Timer.now(); //get the actual time, is needed to calculate z value.

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredCon (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  cloud_filtered.header.frame_id ="VUX-1";


  // Convert to PCL data type
  pcl::fromROSMsg(*input, *cloud_unfiltered);

  //Check if the cloud has points in it
  if(cloud_unfiltered->width > 0 || headingAngle < MAX_HEADING_ANGLE){

  // Perform first filtering => filter all points that are hihger then the MIN_FLIGHT_HIGH under the UAV
  pcl::PassThrough<pcl::PointXYZ> ptfilter (new pcl::PassThrough<pcl::PointXYZ>);
  ptfilter.setInputCloud(cloud_unfiltered);
  ptfilter.setFilterFieldName ("y");
  ptfilter.setFilterLimits (-1000.0, - MIN_FLIGHT_HIGH);
  ptfilter.filter(*cloud_filteredCon );

  //Check again if this cloud has no points in it, the UAV is flying to low.
  if(cloud_filteredCon ->width >0){

  // Preform second filtering => filter all points that don't have any direct neighbours, which are probably dust.
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filter (new pcl::RadiusOutlierRemoval<pcl::PointXYZ>);
  ror_filter.setMinNeighborsInRadius(1);
  ror_filter.setRadiusSearch(0.2);
  ror_filter.setInputCloud(cloud_filteredCon );
  ror_filter.filter(cloud_filtered);

  //Check a last time if the cloud has still enough points in it
  if(cloud_filtered.width > LANDING_FIELD_SIZE * DENSITY_PER_M){

  //Calculate the size of the lineArray which will be computet depending on x position of first and last entry of the given PC.
  int numberOfLines = (int) (abs(cloud_filtered[0].x) + abs(cloud_filtered[cloud_filtered.width-1].x))/LINE_PIECE_SIZE;


#if (IS_SIMULATION ==1)
  std::cout << "X start:" << cloud_filtered[0].x << " X end:" << cloud_filtered[cloud_filtered.width-1].x << endl;
  line_p lineArray (new line_t[numberOfLines]);

#else
  line_p lineArray = new line_t[numberOfLines];
#endif

  //Divide in defined pc pieces and convert them into line pieces
  int pcIterator = 0;
  int lineArrayIterator = 0;

  int pos = (int) cloud_filtered[pcIterator].x;

  do{    
    int start = pcIterator;
    int xStart = pos; //is needed later for calculation of the slop value of the line
    pos = pos + (int) LINE_PIECE_SIZE;

  //find all points that are in one line
  while((cloud_filtered[pcIterator].x < pos) && (pcIterator < (cloud_filtered.width-1))){
    pcIterator ++;
  }
    int end = pcIterator -1;

    int value = 0;
    double yStart = 0;
    double slope = 0;
    float roughness = 0.0;
    int size = abs(end - start)+1;

    if(voltageLinePositionX == 0){
     // std::cout << "DANGER \t !!! line tracking isn't working !!! \n It is possible that landing fields will be detected under the voltage line !!!" << endl;
    }
    else{   //check if the new line is under the voltage line -> make size = 0 => line will considerer as bad because of density
      if((xStart > voltageLinePositionX-VOLTAGE_LINE_SIZE/2 && xStart < voltageLinePositionX+VOLTAGE_LINE_SIZE/2) ||
         (pos > voltageLinePositionX-VOLTAGE_LINE_SIZE/2 && pos < voltageLinePositionX+VOLTAGE_LINE_SIZE/2)){
          size = 0;
      }
    }


    if(size < (DENSITY_PER_M/LINE_PIECE_SIZE)){ // if the densitiy is too small
      value = -1;
    }
    else{
#if (IS_SIMULATION == 7)
    // std::cout <<"b" <<endl;
#endif

      //calculating the slope and yStart value with the least square methode
      double xMean = 0.0;
      double yMean = 0.0;
      double xSquared = 0.0;
      double xyMul = 0.0;

      for(int i = start; i<=end; i++){
        xMean += cloud_filtered[i].x - xStart; //this way every x value of the array is treated as the line woud begin at x = 0
        yMean += cloud_filtered[i].y;
        xSquared += (cloud_filtered[i].x-xStart)*(cloud_filtered[i].x -xStart);
        xyMul += (cloud_filtered[i].x - xStart) * cloud_filtered[i].y;
      }

      xMean = xMean/((float)size);

      yMean = yMean/((float)size);


      slope = (xyMul - (size*xMean*yMean))/(xSquared - (size*xMean*xMean));
      yStart = yMean - (slope*xMean);

      //calculate the average roughness
      for(int i = start ; i <= end; i++){
        float onePointError = abs((slope * (cloud_filtered[i].x -xStart) + yStart - cloud_filtered[i].y));

        if(onePointError > MAX_ACCEPTED_DIFFERENCE){  //Test if the Error is greater then the maximal axepted Difference
          value = -1;  //If there are too much height difference in more then one point -> probebly a staff or something
        }
        roughness += onePointError;
      }
      roughness = roughness/size;

      if(value > -1){
        //Use the valuing function to find the right value of this line
        value = MAX_VALUE_LINE - (roughness*100 + abs(slope*((MAX_VALUE_LINE-MIN_VALUE)/MAX_ACCEPTED_SLOPE))); //So it dosn't become a negative value if slope is negatve. Slope is in percent.
      }

    }

#if (IS_SIMULATION == 1)
    //just for debugging purpouse
    std::cout << "x: " <<x<<" slope: "<< slope<<" ystart: "<<yStart <<" roughness: " <<roughness << " value: " << value <<endl;
    std::cout <<"xstart:" << xstart <<" size:" << size <<" numberOfLines:" <<numberOfLines<< " lineArrayIterator: "<<lineArrayIterator <<endl;
#endif

    //Save calculated values into the Array
    lineArray[lineArrayIterator].x = xStart;
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


zValue += (timestamp.toSec() - oldTime) * currentSpeed;
double z = zValue;
oldTime = timestamp.toSec();


#if (IS_SIMULATION == 1 ||IS_SIMULATION == 6)

    //Convert into line pieces and publish it.

  if((timestamp.toSec() - oldTimeForVis) * currentSpeed > 0.5){
    oldTimeForVis = timestamp.toSec();

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
#if (IS_SIMULATION != 2)
    p.z = z;
#else
    p.z = 0;
#endif

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
#endif

#if (IS_SIMULATION == 2)
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
  line_list_possible.color.r = 0.6;
  line_list_possible.color.g = 0.55;
  line_list_possible.color.a = 1.0;

  // Make good Lines green
  line_list_good.color.g = 1.0f;
  line_list_good.color.a = 1.0;

  geometry_msgs::Point p;

  //determite if the lines are considered good possible or bad

  for(int i = 0; i < numberOfLines; i++){

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

  linePub.publish(line_list_good);
  linePub.publish(line_list_possible);
  linePub.publish(line_list_bad);

#endif

#if IS_SIMULATION
    //Publish the filtered cloud vor visualization purpose
    sensor_msgs::PointCloud2 output;
    output.header.frame_id = "VUX-1";
    pcl::toROSMsg(*cloud_filteredCon, output);

    pub.publish(output);
#endif
#if (IS_SIMULATION == 1 || IS_SIMULATION == 2)
    return;
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
    fieldDetector(calculatedRow);

  }
  else{
    std::cout << "Filtered Cloud is to small for computing !!" << endl;
  }
  }
  else{
    std::cout << "Sensor is too close to the ground. Can't scan for landingfields !!" << endl;
  }
  }
  else{
    std::cout << "Inputcloud was empty !!" << endl;
  }
}


/*\brief stores the given lineRow and checks if there are enough lineRows to compute to find landingfields.
 * If so it searchers through the stored lineRows and deletes halfe of them
 *
 *\param lineRow: a pointer to a lineRow struct whicht should be stored.
 */
static void fieldDetector(lineRow_p lineRow){
   static int lineRowArrayIndexerStart = 0;
   static int lineRowArrayIndexerEnd = 0;
   static int currentEntries = 0;

   //Delet all entries if the lineRowArray is full -> the UAV does't fly forward and therefor it doesn't make any sence to try to find landing fields.
   if(currentEntries == SIZE_OF_ROW_BUFFER){
     deleteEntriesFlag = 1;
   }

   //Is done this way because the flag can also be set from the TurnOccured function to delet all entries.
   if(deleteEntriesFlag){
     deleteEntriesFlag = 0;
     int i = 0;
     int currentEntriesTemp = currentEntries;
     while(i <currentEntriesTemp){

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
        cout <<"entries were deletet:"<< currentEntries << " " << lineRowArrayIndexerEnd << " " << lineRowArrayIndexerStart << endl;
   }

   lineRowArray[lineRowArrayIndexerEnd] = lineRow;
   currentEntries ++;

#if (IS_SIMULATION == 7)
    //std::cout <<"e" <<endl;
#endif

//   float aproxSpeed = (lineRowArray[lineRowArrayIndexerEnd]->velocity +  lineRowArray[lineRowArrayIndexerStart]->velocity)/2;
   float distanceStartEnd = lineRowArray[lineRowArrayIndexerEnd]->z - lineRowArray[lineRowArrayIndexerStart]->z;
#if (IS_SIMULATION == 3 || IS_SIMULATION == 4 )
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
     int contiguouslyGoodLines[maxPossLandingField][2]; //This way it is alway great enough.
     int posLineCounter = 0;
     int i = 0;
     int goodLineGroupsCounter = 0;
     float yStartDifference = 0;
//#if (IS_SIMULATION == 7)
//    std::cout <<"e.1" <<endl;
//#endif
     do{
        // Calculate the difference in hight bewtween the different lines.
        if(i == 0){
          yStartDifference = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].yStart - lineRowArray[lineRowArrayIndexerStart]->LineRow[i+ 1].yStart);
        }
        else{
          yStartDifference = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i-1].yStart - lineRowArray[lineRowArrayIndexerStart]->LineRow[i].yStart);
        }

        //Check the lines if there are enough good lines for a landingfield
        if(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > MIN_VALUE && yStartDifference < MAX_ACCEPTED_DIFFERENCE){
          posLineCounter ++;
          if(posLineCounter > (LANDING_FIELD_SIZE/LINE_PIECE_SIZE)){
            contiguouslyGoodLines[goodLineGroupsCounter][0] = lineRowArray[lineRowArrayIndexerStart]->LineRow[i].x;
            contiguouslyGoodLines[goodLineGroupsCounter][1] = i;
#if (IS_SIMULATION == 3 || IS_SIMULATION == 4)
            std::cout << "posLandingSidesCounter: " << posLandingSidesCounter<<  endl;
#endif
#if (IS_SIMULATION == 3)

            std::cout << "possibleLandingsideDetected at: " << possibleLandingSides[posLandingSidesCounter][0] << endl;
#endif
            goodLineGroupsCounter ++;
            // This way a flat place counts as just one plane not multible ones which enables to find fields that are greater then just the LANDING_FIELD_SIZE
            while(lineRowArray[lineRowArrayIndexerStart]->LineRow[i].value > MIN_VALUE && yStartDifference < MAX_ACCEPTED_DIFFERENCE && i<lineRowArray[lineRowArrayIndexerStart]->numberOfLines)
            {
              i++;
              yStartDifference = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[i-1].yStart - lineRowArray[lineRowArrayIndexerStart]->LineRow[i].yStart);
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
#if (IS_SIMULATION != 1 || IS_SIMULATION != 2)
      // Check if there are possible landingsides which can be computed
      if(goodLineGroupsCounter > 0){
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
          while(goodColumns){
            if(lineRowArray[lineRowArrayIndexerStart]->LineRow[contiguouslyGoodLines[i][1]+indexer].value < MIN_VALUE){
              goodColumns = 0;
            }
            //find the same line in the upper rows an check there values
              upLinesChecker = 1; // Flag for detecting bad Lines in the checked field if 0 => one or more bad lines were detected or there were no lines to check.

              int y = lineRowArrayIndexerStart + 1;
              if(y >= SIZE_OF_ROW_BUFFER) {y = 0;}
              int z = 0;
              while(y != lineRowArrayIndexerEnd && (upLinesChecker || !indexer)){ //this loop iterates from the given start line upwards in lineRows
              if(indexer == 0){ //if this is the first iteration for this possibleLandingSides entry
                int j = 0;
                while(lineRowArray[y]->LineRow[j].x != contiguouslyGoodLines[i][0] && upLinesChecker){  //find the line in the upper lineRow with the same x position as the one at the bottom of the Array
                  j++;
                  if(j >= (lineRowArray[y]->numberOfLines -1)){upLinesChecker = 0;}
                }
                startingPoints[z] = j;      //Store the actual index into the Array to find the same point faster in the next iteration
                if(upLinesChecker){
                  if(lineRowArray[y]->LineRow[j].value < MIN_VALUE) {upLinesChecker = 0;}
                  else{
                    // Save the amount of lines an there value for later use
                    landingFieldSize ++;
                    landingFieldValue += lineRowArray[y]->LineRow[j].value;
                  }
                }
              }
              else{ //same as above if it isn't the first iteration
                if(startingPoints[z]+indexer < lineRowArray[y]->numberOfLines){
                  if(lineRowArray[y]->LineRow[startingPoints[z]+indexer].value < MIN_VALUE) {upLinesChecker = 0;} // if the value from just one line is considered as bad.
                  else{
                    landingFieldSize ++;
                    landingFieldValue += lineRowArray[y]->LineRow[startingPoints[z]+indexer].value;
                  }
                }
                else{
                  upLinesChecker = 0;
                }
              }

#if (IS_SIMULATION == 3)
            std::cout << "y: " << y<<  "\t j: " << j<<  endl;
            std::cout <<     " z: " << z <<
                             "\t y: " << y <<
                             "\t startingPoint: " << startingPoints[z] <<
                             "\t indexer: " << indexer <<
                             "\t numOfLi: " << lineRowArray[y]->numberOfLines << endl;

#endif
              y ++;
              if(y >= SIZE_OF_ROW_BUFFER) {y = 0;}
              z ++;
            }
            if(upLinesChecker){ // if all upper lines were also good.
              float columnSlope = abs(lineRowArray[lineRowArrayIndexerStart]->LineRow[contiguouslyGoodLines[i][1]+indexer].yStart - lineRowArray[lineRowArrayIndexerEnd]->LineRow[startingPoints[z-1]+indexer].yStart)/LANDING_FIELD_SIZE;
              if(columnSlope < MAX_ACCEPTED_SLOPE){ //check if slope in z-axis is also good.
                goodColumnsCounter ++;
                slope += columnSlope;
              }
            }
            else if(goodColumnsCounter < LANDING_FIELD_SIZE/LINE_PIECE_SIZE){
              landingFieldSize = 0;
              landingFieldValue  = 0;
              goodColumnsCounter = 0;
              slope = 0;
            }
            if(goodColumnsCounter >= (LANDING_FIELD_SIZE/LINE_PIECE_SIZE) && !upLinesChecker){ //Enough good Columns where detectet -> ah possible landing field
              //publish here a possible landing field with the array of the lines -> value, time, speed etc.

              //make a container for the LandingField and fill it with the right values:
              pcl_filter::LandingField newPossLandField;
              //check if the density in Z axis is at least 2 line per meter
              if(landingFieldSize/goodColumnsCounter > LANDING_FIELD_SIZE * 2){
              slope = slope/goodColumnsCounter;
              //calculate the value with the value functione. Should maybe done differently
              newPossLandField.value = (landingFieldValue/landingFieldSize) - (slope * 100) + (goodColumnsCounter - LANDING_FIELD_SIZE/LINE_PIECE_SIZE)*SIZE_BONUS;
              int middleLine = lineRowArrayIndexerStart + currentEntries/2;
              if(middleLine >= SIZE_OF_ROW_BUFFER-1){
                middleLine = currentEntries/2 - (SIZE_OF_ROW_BUFFER-1-lineRowArrayIndexerStart);
              }
              newPossLandField.timestamp = lineRowArray[middleLine]->timestamp;
              newPossLandField.velocity = lineRowArray[middleLine]->velocity;
              newPossLandField.z = lineRowArray[middleLine]->z;
              newPossLandField.hight = lineRowArray[lineRowArrayIndexerStart]->LineRow[contiguouslyGoodLines[i][1]+(int)(goodColumnsCounter/2)].yStart;
              newPossLandField.width = LANDING_FIELD_SIZE;
              newPossLandField.turnCounter = turnCounter;

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

              landingFieldSize = 0;
              landingFieldValue  = 0;
              goodColumnsCounter = 0;
              slope = 0;
              }
            }


            indexer++;
          }
          i++;
        }
        while(i < goodLineGroupsCounter);
      }
#endif

#if (IS_SIMULATION == 3)
      linePub.publish(line_list_good);

#endif


      //delete the oldest few entries

#if (IS_SIMULATION == 4)

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
   if(lineRowArrayIndexerEnd >= SIZE_OF_ROW_BUFFER){
     lineRowArrayIndexerEnd = 0;
   }
#if (IS_SIMULATION == 2 || IS_SIMULATION == 4)

            std::cout << "loop finished" << endl;
#endif
}

static void updateVoltageLinePos(std_msgs::Int32 xDistance){
  voltageLinePositionX = xDistance.data/100;
}

static void updateVelocity(std_msgs::Int32 newVelocity){
  currentSpeed = newVelocity.data;
}

static void updateHeadingAngle(std_msgs::Int32 newAngle){
  headingAngle = newAngle.data;
}

static void turnOccured(std_msgs::Int32 occuredTurn){
  turnCounter ++;
  oldTime = Timer.now().toSec();
  oldTimeForVis = oldTime;
  zValue = 0;
  deleteEntriesFlag = 1;


#if(IS_SIMULATION == 7)
//  cout << "Turn occrued:  " << turnCounter << endl;
#endif

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)
  //empty the list
  line_list_bad.points.clear();
  line_list_possible.points.clear();
  line_list_good.points.clear();
#endif
}

#if (IS_SIMULATION == 1 || IS_SIMULATION == 6)
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
  ros::Subscriber sub3 = n.subscribe("UAV_velocity",1,updateVelocity);
  ros::Subscriber sub4 = n.subscribe("UAV_headingAngle",1,updateHeadingAngle);
  ros::Subscriber sub5 = n.subscribe("Turn_notification",1,turnOccured);
// %EndTag(SUBSCRIBER)%


  // Create a ROS publisher for the output point cloud
#if IS_SIMULATION
  pub = n.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_from_linefilter", 1);
  linePub = n.advertise<visualization_msgs::Marker>("filteredRowinLines", 1);
#endif
  possibleLandingFieldsPub = n.advertise<pcl_filter::LandingField>("possLandingFields",1);


  Timer = ros::Time(0.0);
  oldTime = Timer.now().toSec();
  oldTimeForVis = oldTime;


 // this is just for visualisation
#if (IS_SIMULATION == 1 || IS_SIMULATION == 6 )
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

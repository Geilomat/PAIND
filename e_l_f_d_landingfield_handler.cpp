
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
// PCL specific includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>

#include <pcl_filter/LandingField.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include "e_l_f_d.h"

//specific include for the columns
//#include "linerow.h"




//possibleLandingField_p* posLandingFieldArray; //Array where the possible landing sides are stored;
ros::Publisher LandingFields;
possibleLandingField_t posLandingFieldArray[NUMBER_OF_POSSIBLE_LANDING_FIELDS];
float currentSpeed = 5.0;       //Current speed of the Drone needs to be updated ! [mm]
//int minValue = 500;              //min value which each line needs to have to be considered as good
int currentEntries = 0;

/*\brief fuses to neighbouring LandingFields and saves them into the pointer from the first !!
 *
 * */
static void mergeTwoLandingFields(possibleLandingField_p toBeMergedInto, possibleLandingField_p toBeMergedFrom){
//  possibleLandingField_t mergedField;

//  mergedField.mergeCounter = toBeMergedInto->mergeCounter + 1;

//  //merge the fields
//  mergedField.xPos = (toBeMergedInto->xPos * (toBeMergedInto->mergeCounter +1) + toBeMergedFrom->xPos)/(2+toBeMergedInto->mergeCounter);
//  mergedField.time = toBeMergedInto->time;
//  mergedField.hight = (toBeMergedInto->hight * (toBeMergedInto->mergeCounter +1)+ toBeMergedFrom->hight)/(2+toBeMergedInto->mergeCounter);
//  mergedField.speed = (toBeMergedInto->speed * (toBeMergedInto->mergeCounter +1)+ toBeMergedFrom->speed)/(2+toBeMergedInto->mergeCounter);
//  mergedField.z = (toBeMergedInto->z * (toBeMergedInto->mergeCounter +1) + toBeMergedFrom->z)/(2+toBeMergedInto->mergeCounter);
//  int value = (toBeMergedInto->initValue + toBeMergedFrom->initValue)/2 + (50 * toBeMergedInto->mergeCounter +1); //Bonus cause of merging
//  //if(value > MAX_VALUE) value = MAX_VALUE;
//  mergedField.initValue = value;
//  mergedField.value = mergedField.initValue; //reset the value cause of merging


  toBeMergedInto->mergeCounter ++;
  toBeMergedInto->xPos = (toBeMergedInto->xPos * (toBeMergedInto->mergeCounter +1) + toBeMergedFrom->xPos)/(2+toBeMergedInto->mergeCounter);
  toBeMergedInto->hight = (toBeMergedInto->hight * (toBeMergedInto->mergeCounter +1)+ toBeMergedFrom->hight)/(2+toBeMergedInto->mergeCounter);
  toBeMergedInto->speed = (toBeMergedInto->speed * (toBeMergedInto->mergeCounter +1)+ toBeMergedFrom->speed)/(2+toBeMergedInto->mergeCounter);
  toBeMergedInto->z =  (toBeMergedInto->z * (toBeMergedInto->mergeCounter +1) + toBeMergedFrom->z)/(2+toBeMergedInto->mergeCounter);
  toBeMergedInto->value = (toBeMergedInto->initValue + toBeMergedFrom->initValue)/2 + (50 * (toBeMergedInto->mergeCounter +1));
  toBeMergedInto->initValue = toBeMergedInto->value;

  std::cout << "merged: X: "<< toBeMergedInto->xPos <<
               " Z: " << toBeMergedInto->z <<
               "\t and: X: " << toBeMergedFrom->xPos<<
               " Z: " <<toBeMergedFrom->z << endl;
             // "\t into: X " << mergedField.xPos<<
             //  " Z: " << mergedField.z<< endl;
  //save the new values into the first parameter
  //*toBeMergedInto = mergedField;
}





static void addNewLandingField(pcl_filter::LandingField landingField){
  //create a container for the data and fill it


#if (IS_SIMULATION == 4)
  std::cout << "LandingField recieved !! \n"
              "value: " << landingField.value <<
              "\t xPos: " << landingField.xPos <<
              "\t time: " << landingField.timestamp <<
              "\t speed: " << landingField.velocity <<
              "\t hight: " << landingField.hight <<
              "\t length: " << landingField.length << endl;
  std::cout << "currentEntriesNewLine. " << currentEntries <<endl;
#endif

  possibleLandingField_t inputLandingField;
  inputLandingField.initValue = landingField.value;
  inputLandingField.value = landingField.value;
  inputLandingField.speed = landingField.velocity;
  inputLandingField.xPos = landingField.xPos;
  inputLandingField.time = landingField.timestamp;
  inputLandingField.z = landingField.z;
  inputLandingField.hight = landingField.hight;
  inputLandingField.length = landingField.length;
  inputLandingField.mergeCounter = 0;

#if (IS_SIMULATION == 5)

  visualization_msgs::Marker posLandingField;
  posLandingField.type = visualization_msgs::Marker::LINE_LIST;
  posLandingField.lifetime = ros::Duration(2,0);
  posLandingField.header.frame_id = "VUX-1";
  posLandingField.header.stamp = ros::Time::now();
  posLandingField.ns = "points_and_lines";
  posLandingField.action =visualization_msgs::Marker::ADD;
  posLandingField.pose.orientation.w = 1.0;
  posLandingField.id = 0;

  // Set width of lines
  posLandingField.scale.x = 0.5;


  // Make good Lines green
  posLandingField.color.g = 1.0f;
  posLandingField.color.a = 1.0;

  geometry_msgs::Point p;
  p.z = 0;

  p.x = inputLandingField.xPos  - LANDING_FIELD_SIZE/2;
  p.y = inputLandingField.hight;
  posLandingField.points.push_back(p);

  p.x = inputLandingField.xPos  + LANDING_FIELD_SIZE/2;
  posLandingField.points.push_back(p);

  LandingField1.publish(posLandingField);
#endif

  //iterate trough the array and fing the right place for the given landingField.

  if(currentEntries == 0){
    posLandingFieldArray[0] = inputLandingField;
    currentEntries ++;
  }
  else{

    //search throu the landingField array to find mergable landingfields and if so merge them.

//    for(int j = 0; j < currentEntries; j++){

//      int zDistance = inputLandingField.z - posLandingFieldArray[j].z; // inputLandingFields z value should always be same or greater then the entry form the array.
//      if(zDistance <= LANDING_FIELD_SIZE){
//        float x
//      }
//    }

//      int xDistance = abs(posLandingFieldArray[j].xPos - inputLandingField.xPos);
//      if(xDistance <= LANDING_FIELD_SIZE){

//       int zDistance = inputLandingField.z - posLandingFieldArray[j].z; // inputLandingFields z value should always be same or greater then the entry form the array.
//       if(zDistance <= LANDING_FIELD_SIZE){
//         mergeTwoLandingFields(&posLandingFieldArray[j], &inputLandingField);
//         return;
//       }
//      }
//    }



    int i = 0;
    //find the right place for the calculated landing Field.
    while(i < currentEntries && posLandingFieldArray[i].value > inputLandingField.value ){
      i ++;
      if(i >= NUMBER_OF_POSSIBLE_LANDING_FIELDS){
        //all landingFields current in the array are better then the one calculated
        return;
      }
    }
    if(i < currentEntries){
      //put the inputLandingField into the right place:
      possibleLandingField_t tempLandingField;
      tempLandingField = posLandingFieldArray[i];
      posLandingFieldArray[i] = inputLandingField;
      i++;

      //push the other entries deeper back into the Array
      possibleLandingField_t tempLandingField2;

      for(i ; i< currentEntries; i++){
        tempLandingField2 = posLandingFieldArray[i];
        posLandingFieldArray[i] = tempLandingField;
        tempLandingField = tempLandingField2;
      }
      //if the end of the array is reached delet the last entry, otherwise put the last entry at the first free place an count the currentEntries up.
      if(currentEntries < NUMBER_OF_POSSIBLE_LANDING_FIELDS){
        posLandingFieldArray[currentEntries] = tempLandingField;
        currentEntries ++;
        }
      }
    else{
      posLandingFieldArray[currentEntries] = inputLandingField;
      currentEntries ++;
    }
  }
}


static void updateCurrentSpeed(std_msgs::Int32 newSpeed){
  currentSpeed = newSpeed.data;
}


static void updateLandingFieldArray(const ros::TimerEvent& event){

#if (IS_SIMULATION == 5)
  //std::cout << "updateLandingFieldArray was called" << endl;
    std::cout << "currentEntriesUpdate " << currentEntries <<endl;
#endif
  // Function is only needed if there are more then one landing field in the array
  if(currentEntries > 1){
    //Use forgettingfunction to give the entries new values
    for(int i = 0; i<currentEntries ; i++){
      int newValue = posLandingFieldArray[i].value - (currentEntries * 2 + currentSpeed + (MAX_VALUE - posLandingFieldArray[i].initValue)/10);
      if(newValue > MIN_VALUE && newValue < MAX_VALUE){
        posLandingFieldArray[i].value = newValue;
      }
      else{
        posLandingFieldArray[i].value = MIN_VALUE -1;
      }

    }
    possibleLandingField_t tempLandingField;
    //sort the Array and delete "bad" entries
    int sorted = 0;
    while(sorted < currentEntries-1){
      for(int i = 0;i<currentEntries-1;i++){
        if(posLandingFieldArray[i].value < posLandingFieldArray[i+1].value){
          tempLandingField = posLandingFieldArray[i];
          posLandingFieldArray[i] = posLandingFieldArray[i+1];
          posLandingFieldArray[i+1] = tempLandingField;
        }
        else{
          sorted ++;
        }
      }

    }
    int i = currentEntries -1;
    while(posLandingFieldArray[i].value < MIN_VALUE && i > 0){
        posLandingFieldArray[i].value = 0; // fill pointer with NULL for saftey reasons
        currentEntries --;
        i --;
    }
  }
#if (IS_SIMULATION > 0)
  std::cout <<"posLandingFieldArray entries:" << endl;
  for(int i= 0 ; i < currentEntries; i++){
  std::cout <<"value: " << posLandingFieldArray[i].value <<
              "\tinitValue: " << posLandingFieldArray[i].initValue<<
              "\t xPos: " << posLandingFieldArray[i].xPos <<
              //"\t time: " << posLandingFieldArray[i].time <<
              "\t velocity: " << posLandingFieldArray[i].speed <<
              "\t z: "  << posLandingFieldArray[i].z<<
              "\t hight: " << posLandingFieldArray[i].hight <<
              "\t length: " << posLandingFieldArray[i].length <<
              "\t mergeCounter: "<< posLandingFieldArray[i].mergeCounter << endl;
  }

#endif
#if(IS_SIMULATION == 6)
  visualization_msgs::MarkerArray posLandingFields;
  posLandingFields.markers.resize(currentEntries);

  for(int i = 0; i < currentEntries; i++){

  //visualization_msgs::Marker posLandingField;
  posLandingFields.markers[i].type = visualization_msgs::Marker::CUBE;
  posLandingFields.markers[i].lifetime = ros::Duration(5,0);
  posLandingFields.markers[i].header.frame_id = "VUX-1";
  posLandingFields.markers[i].header.stamp = ros::Time::now();
  posLandingFields.markers[i].ns = "posLandingField";
  posLandingFields.markers[i].action =  visualization_msgs::Marker::ADD;
  posLandingFields.markers[i].pose.orientation.w = 1.0;
  posLandingFields.markers[i].id = i;

  // Set width of lines
  posLandingFields.markers[i].scale.x = posLandingFieldArray[i].length;
  posLandingFields.markers[i].scale.y = 0.2;
  posLandingFields.markers[i].scale.z = LANDING_FIELD_SIZE;


  // Make them blue
  posLandingFields.markers[i].color.b = 0.5f;
  posLandingFields.markers[i].color.a = 0.6;

  // Set position
  posLandingFields.markers[i].pose.position.x = posLandingFieldArray[i].xPos;
  posLandingFields.markers[i].pose.position.y = posLandingFieldArray[i].hight;
  posLandingFields.markers[i].pose.position.z = posLandingFieldArray[i].z;

  }
  LandingFields.publish(posLandingFields);
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
  ros::init(argc, argv, "e_l_f_d_landingfield_handler");

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
  ros::Subscriber sub = n.subscribe("possLandingFields",10,addNewLandingField);
  //ros::Subscriber sub4 = n.subscribe("UAV_velocity",1,updateVelocity);
// %EndTag(SUBSCRIBER)%
  ros::Timer timer = n.createTimer(ros::Duration(5),updateLandingFieldArray,false);
  // Create a ROS publisher for the output point cloud
 // posLandingFieldArray = new possibleLandingField_p[NUMBER_OF_POSSIBLE_LANDING_FIELDS];
#if (IS_SIMULATION == 5)
  LandingFields = n.advertise<visualization_msgs::Marker>("LandingFields", 1);
#elif (IS_SIMULATION == 6)
  LandingFields = n.advertise<visualization_msgs::MarkerArray>("LandingFields", 1);
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

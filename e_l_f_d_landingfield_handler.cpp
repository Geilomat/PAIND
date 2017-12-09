
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
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



ros::Publisher LandingFields;
possibleLandingField_t landingFieldArray[NUMBER_OF_POSSIBLE_LANDING_FIELDS];
float currentSpeed = 0.0;       //Current speed of the Drone needs to be updated ! [mm]
int currentEntries = 0;
char textFileName[50];

/*\brief merges to neighbouring LandingFields and saves them into the pointer from the first !!
 *
 * */
static void mergeTwoLandingFields(possibleLandingField_p arrayLF, possibleLandingField_p inputLF){

#if (IS_SIMULATION == 5)
  std::cout << "merge: X: "<< toBeMergedInto->xPos <<
               "\t Z: " << toBeMergedInto->z <<
               "\t value: " << toBeMergedInto->value <<
               "\t lenght: " << toBeMergedInto->length <<
               "\t width: " << toBeMergedInto->width <<
               "\n and: X: " << toBeMergedFrom->xPos<<
               "\t Z: "    << toBeMergedFrom->z <<
               "\t value: " << toBeMergedFrom->value <<
               "\t lenght: " << toBeMergedFrom->length <<
               "\t width: " << toBeMergedFrom->width << endl;
#endif

  arrayLF->mergeCounter ++;
  arrayLF->xPos = (arrayLF->xPos * arrayLF->mergeCounter + inputLF->xPos)/(1+arrayLF->mergeCounter);
  arrayLF->length = (arrayLF->length * arrayLF->mergeCounter + inputLF->length)/(1+arrayLF->mergeCounter);
  int additionalWidth = inputLF->width - ((arrayLF->z + arrayLF->width/2) -(inputLF->z - inputLF->width/2));
  arrayLF->width = arrayLF->width + additionalWidth;
  arrayLF->speed = (arrayLF->speed * arrayLF->mergeCounter + inputLF->speed)/(1+arrayLF->mergeCounter);
  arrayLF->z =  (arrayLF->z * arrayLF->mergeCounter + inputLF->z)/(1+arrayLF->mergeCounter);
  arrayLF->value = (arrayLF->initValue * arrayLF->mergeCounter + inputLF->initValue)/(1 + arrayLF->mergeCounter) + (SIZE_BONUS * additionalWidth);
  arrayLF->initValue = arrayLF->value;

#if (IS_SIMULATION == 5)
  std::cout << "into : X: "<< toBeMergedInto->xPos <<
               "\t Z: " << toBeMergedInto->z <<
               "\t value: " << toBeMergedInto->value <<
               "\t lenght: " << toBeMergedInto->length <<
               "\t width: " << toBeMergedInto->width << endl;

#endif
}





static void addNewLandingField(pcl_filter::LandingField landingField){
  //create a container for the data and fill it

#if (IS_SIMULATION == 4)
  std::cout << "LandingField recieved !! \n"
              "value: " << landingField.value <<
              "\t xPos: " << landingField.xPos <<
              "\t time: " << landingField.timestamp <<
              "\t speed: " << landingField.velocity <<
              "\n z: "    <<landingField.z <<
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
  inputLandingField.height = landingField.hight;
  inputLandingField.length = landingField.length;
  inputLandingField.width = landingField.width;
  inputLandingField.turnCounter = landingField.turnCounter;
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

  LandingFields.publish(posLandingField);
#endif

  //iterate trough the array and fing the right place for the given landingField.

  if(currentEntries == 0){
    landingFieldArray[0] = inputLandingField;
    currentEntries ++;
  }
  else{

    //search throu the landingField array to find mergable landingfields and if so merge them.

    for(int j = 0; j < currentEntries; j++){
      if(landingFieldArray[j].mergeCounter < 5 && landingFieldArray[j].turnCounter == inputLandingField.turnCounter){
        int zDistance = inputLandingField.z - landingFieldArray[j].z; // inputLandingFields z value should always be same or greater then the entry form the array.
        if(zDistance <= (landingFieldArray[j].width/2 + LANDING_FIELD_SIZE/2)){ //check if z distanzc isn't too far
          float xDistance =  abs(landingFieldArray[j].xPos - inputLandingField.xPos);
          if(xDistance <= LANDING_FIELD_SIZE){ //Check if x distance is also acceptable
            float lengthDiff = abs(landingFieldArray[j].length - inputLandingField.length);
            if(lengthDiff < LANDING_FIELD_SIZE){ //Ckeck if there is a maxiaml length difference of 2 min sized LandingFields
              mergeTwoLandingFields(&landingFieldArray[j], &inputLandingField);
              return;
            }
          }
        }
      }
    }



    int i = 0;
    //find the right place for the calculated landing Field.
    while(i < currentEntries && landingFieldArray[i].value > inputLandingField.value ){
      i ++;
      if(i >= NUMBER_OF_POSSIBLE_LANDING_FIELDS){
        //all landingFields current in the array are better then the one calculated
        return;
      }
    }
    if(i < currentEntries){
      //put the inputLandingField into the right place:
      possibleLandingField_t tempLandingField;
      tempLandingField = landingFieldArray[i];
      landingFieldArray[i] = inputLandingField;
      i++;

      //push the other entries deeper back into the Array
      possibleLandingField_t tempLandingField2;

      for(i ; i< currentEntries; i++){
        tempLandingField2 = landingFieldArray[i];
        landingFieldArray[i] = tempLandingField;
        tempLandingField = tempLandingField2;
      }
      //if the end of the array is reached delet the last entry, otherwise put the last entry at the first free place an count the currentEntries up.
      if(currentEntries < NUMBER_OF_POSSIBLE_LANDING_FIELDS){
        landingFieldArray[currentEntries] = tempLandingField;
        currentEntries ++;
        }
      }
    else{
      landingFieldArray[currentEntries] = inputLandingField;
      currentEntries ++;
    }
  }
}




static void updateLandingFieldArray(const ros::TimerEvent& event){


#if (IS_SIMULATION == 5)
    std::cout << "currentEntriesUpdate " << currentEntries <<endl;
#endif
  // Function is only needed if there are more then one landing field in the array
  if(currentEntries > 1){
    //Use forgettingfunction to give the entries new values
    for(int i = 0; i<currentEntries ; i++){
      int newValue = landingFieldArray[i].value - (currentEntries + currentSpeed + (MAX_VALUE_FIELD - landingFieldArray[i].initValue)/10);
      if(newValue > MIN_VALUE && newValue < MAX_VALUE_FIELD){
        landingFieldArray[i].value = newValue;
      }
      else{
        landingFieldArray[i].value = MIN_VALUE -1;
      }

    }
    //sort the Array and delete "bad" entries
    possibleLandingField_t tempLandingField;
    int sorted = 0;
    while(sorted < currentEntries-1){
      for(int i = 0;i<currentEntries-1;i++){
        if(landingFieldArray[i].value < landingFieldArray[i+1].value){
          tempLandingField = landingFieldArray[i];
          landingFieldArray[i] = landingFieldArray[i+1];
          landingFieldArray[i+1] = tempLandingField;
        }
        else{
          sorted ++;
        }
      }

    }
    int i = currentEntries -1;
    while(landingFieldArray[i].value < MIN_VALUE && i > 0){
        landingFieldArray[i].value = 0;
        currentEntries --;
        i --;
    }
  }
#if (IS_SIMULATION > 0)
  std::cout <<"posLandingFieldArray entries:" << endl;
  for(int i= 0 ; i < currentEntries; i++){
  std::cout <<"value:" << landingFieldArray[i].value <<
              "\tinitVal:" << landingFieldArray[i].initValue<<
              "\txPos:" << landingFieldArray[i].xPos <<
              "\ttime:" << landingFieldArray[i].time <<
              "\tspeed:" << landingFieldArray[i].speed <<
              "\nz:"  << landingFieldArray[i].z<<
              "\thight:" << landingFieldArray[i].height <<
              "\tlength:" << landingFieldArray[i].length <<
              "\twidth:" << landingFieldArray[i].width<<
              "\tmergeCnt:"<< landingFieldArray[i].mergeCounter <<
              "\tturnCnt:" <<landingFieldArray[i].turnCounter <<
              "\n" << endl;
  }




#endif

#if (IS_SIMULATION == 7)


  static int Counter = 0;
  if(currentEntries > 0){
  std::ofstream file_stream;
  file_stream.open(textFileName,ios_base::app);
  file_stream << "Entry Nbr: " << Counter <<
               "\n posLandingFieldArray entries:" << endl;
  for(int i= 0 ; i < currentEntries; i++){
  file_stream <<"Nr." << i + 1 <<
              "\tvalue:" << landingFieldArray[i].value <<
              "\tinitVal:" << landingFieldArray[i].initValue<<
              "\txPos:" << landingFieldArray[i].xPos <<
              "\ttime:" << landingFieldArray[i].time <<
              "\tspeed:" << landingFieldArray[i].speed <<
              "\nz:"  << landingFieldArray[i].z<<
              "\thight:" << landingFieldArray[i].height <<
              "\tlength:" << landingFieldArray[i].length <<
              "\twidth:" << landingFieldArray[i].width <<
              "\tmergeCnt:"<< landingFieldArray[i].mergeCounter <<
              "\tturnCnt:" << landingFieldArray[i].turnCounter <<
              "\n" << endl;
  }
  Counter ++;
  file_stream.close();
  }

#endif
#if(IS_SIMULATION == 6)
  visualization_msgs::MarkerArray posLandingFields;
  posLandingFields.markers.resize(currentEntries*2);

  // Fill the Marker Array with the possible LandingFields in blue
  for(int i = 0; i < currentEntries; i++){
    posLandingFields.markers[i].type = visualization_msgs::Marker::CUBE;
    posLandingFields.markers[i].lifetime = ros::Duration(3,0);
    posLandingFields.markers[i].header.frame_id = "VUX-1";
    posLandingFields.markers[i].header.stamp = ros::Time::now();
    posLandingFields.markers[i].ns = "posLandingField";
    posLandingFields.markers[i].action =  visualization_msgs::Marker::ADD;
    posLandingFields.markers[i].pose.orientation.w = 1.0;
    posLandingFields.markers[i].id = i;

    // Set width of lines
    posLandingFields.markers[i].scale.x = landingFieldArray[i].length;
    posLandingFields.markers[i].scale.y = 0.2;
    posLandingFields.markers[i].scale.z = landingFieldArray[i].width;

    // Make them blue
    posLandingFields.markers[i].color.b = 0.5f;
    posLandingFields.markers[i].color.a = 0.6;

    // Set position
    posLandingFields.markers[i].pose.position.x = landingFieldArray[i].xPos;
    posLandingFields.markers[i].pose.position.y = landingFieldArray[i].height;
    posLandingFields.markers[i].pose.position.z = landingFieldArray[i].z;
  }

  //fill the Markers with the corresponding number in white
  for(int i= 0; i < currentEntries; i++){

    posLandingFields.markers[i+currentEntries].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    posLandingFields.markers[i+currentEntries].lifetime = ros::Duration(3,0);
    posLandingFields.markers[i+currentEntries].header.frame_id = "VUX-1";
    posLandingFields.markers[i+currentEntries].header.stamp = ros::Time::now();
    posLandingFields.markers[i+currentEntries].ns = "posLandingField";
    posLandingFields.markers[i+currentEntries].action =  visualization_msgs::Marker::ADD;
    posLandingFields.markers[i+currentEntries].pose.orientation.w = 1.0;
    posLandingFields.markers[i+currentEntries].id = i+currentEntries;
    std::stringstream number;
    number << i+1;
    posLandingFields.markers[i+currentEntries].text = number.str();

    // Set width of lines
    posLandingFields.markers[i+currentEntries].scale.z = 10;


    // Make them white
    posLandingFields.markers[i+currentEntries].color.b = 1;
    posLandingFields.markers[i+currentEntries].color.r = 1;
    posLandingFields.markers[i+currentEntries].color.g = 1;
    posLandingFields.markers[i+currentEntries].color.a = 1;

    // Set position
    posLandingFields.markers[i+currentEntries].pose.position.x = landingFieldArray[i].xPos;
    posLandingFields.markers[i+currentEntries].pose.position.y = landingFieldArray[i].height;
    posLandingFields.markers[i+currentEntries].pose.position.z = landingFieldArray[i].z;
  }
  LandingFields.publish(posLandingFields);
#endif
}

static void updateCurrentSpeed(std_msgs::Int32 newSpeed){
  currentSpeed = newSpeed.data;
}

static void turnOccured(std_msgs::Int32 turnNbr){
  std::ofstream file_stream;
  file_stream.open(textFileName,ios_base::app);
  file_stream << "\n Turn Occured. Z was set to 0 !! \n " << endl;
  file_stream.close();
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
  ros::Subscriber sub4 = n.subscribe("UAV_velocity",1,updateCurrentSpeed);
  ros::Subscriber sub5 = n.subscribe("Turn_notification",1,turnOccured);
// %EndTag(SUBSCRIBER)%

  // create a timer which calls the sorting-function periodically
  ros::Timer timer = n.createTimer(ros::Duration(3),updateLandingFieldArray,false);


  // Create a ROS publisher for the output poisinglnt cloud
#if (IS_SIMULATION == 5)
  LandingFields = n.advertise<visualization_msgs::Marker>("LandingFields", 1);
#elif (IS_SIMULATION == 6)
  LandingFields = n.advertise<visualization_msgs::MarkerArray>("LandingFields", 1);
#endif

#if (IS_SIMULATION == 7)
  //std::ofstream file_stream;
  //file_stream.open("LandingFields.txt",ios_base::trunc);
  //file_stream.close();

  // concatonate the date onto the file name so that no other text file geds overwritten.
  time_t timeAct;
  time(&timeAct);
  strftime(textFileName,49,"LandingFields_%d_%m_%H_%M",localtime(&timeAct));
  strcat(textFileName,".txt");
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

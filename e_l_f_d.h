#ifndef E_L_F_D_H
#define E_L_F_D_H

#include <pcl_filter/LandingField.h>

#define IS_SIMULATION 7                       /*define which simulation should be running or if it runs on a real system
                                               * 1: Visualization of the line fitting algorithm with upgowing z value
                                               * 2: Visualization of just one row at a time with z value always 0
                                               * 3: Visualization of the landing field detector, landing fields are visualed as green lines in the PC rows. Does just check one Row !!
                                               * 4: For checking the communication between e_l_f_d and e_l_f_d_landingfield_handler as console output.
                                               * 5: For chechking merging and the forgetting function from the e_l_f_d_landingfield_handler as console output.
                                               * 6: Visualization of the whole algorithm with landingfields and their ranking
                                               * 7: Save the calculated landing fields into LandingFields.txt file.
                                               */
#define NUMBER_OF_POSSIBLE_LANDING_FIELDS 10  //max # of possible landing sides which shoulde be stored
#define LINE_PIECE_SIZE 1.0f                  //size of one line piece [m]
#define DENSITY_PER_M 10                      //amount of points per meter which are needet for a line to be considered as possible
#define MAX_ACCEPTED_DIFFERENCE 0.5f          //max accepted difference for one point [m]
#define MAX_ACCEPTED_SLOPE 0.33f              //max accepted slope for the landing field +- 15 grde => +- 0.33m/m
#define VOLTAGE_LINE_SIZE 10                  //horizontal size of the voltage line
#define LANDING_FIELD_SIZE 10                 //minimal size of the squared landing field [m]
#define MAX_VALUE_LINE 1000                   //the maximal possible value for a line
#define MAX_VALUE_FIELD 2000                  //the maximal possible value for a landing field
#define MIN_VALUE 500                         //min value which is needed that a line or landing field is considered as good. 1000 is the Maximal value
#define SIZE_OF_ROW_BUFFER 1000               //size of the ring buffer which stores the line rows. Has to be great enough to store at least the minimal hight of the landing fiels
#define MIN_FLIGHT_HIGH 20                    //the minimal flight hight above which the algorithm will start to try to detect landing fields.

typedef struct line{
  int x;
  float yStart;
  float slope;
  float roughness;
  int value;
}line_t;

typedef line_t* line_p;

typedef struct lineRow{
  line_p LineRow;
  ros::Time timestamp;
  float velocity;
  double z;
  int numberOfLines;
}lineRow_t;

typedef lineRow_t* lineRow_p;

typedef struct possibleLandingField{
  int value;
  int initValue;
  int hight;
  int mergeCounter;
  int length;
  int width;
  int turnCounter;
  ros::Time time;
  float xPos;
  float speed;
  double z;
}possibleLandingField_t;

typedef possibleLandingField_t* possibleLandingField_p;

static void handleLines(lineRow_p);

static void updateVoltageLinePos(std_msgs::Int32);


#endif // E_L_F_D_H

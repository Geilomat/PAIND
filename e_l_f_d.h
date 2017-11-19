#ifndef E_L_F_D_H
#define E_L_F_D_H

#include <pcl_filter/LandingField.h>


#define IS_SIMULATION 4                       //define if this is a simulation or runnning on real time
#define NUMBER_OF_POSSIBLE_LANDING_FIELDS 10  //# of possible landing sides which shoulde be stored
#define LINE_PIECE_SIZE 1.0f                   //size of one line piece [m]
#define DENSITY_PER_M 10                      //amount of points per meter which are needet for a line to be considered as possible
#define MAX_ACCEPTED_DIFFERENCE 0.5f           //max accepted difference for one point [m]
#define MAX_ACCEPTED_SLOPE 0.33f               //max accepted slope for the landing field +- 15 grde => +- 0.33m/m
#define VOLTAGE_LINE_SIZE 10                  //horizontal size of the voltage line
#define LANDING_FIELD_SIZE 10                 //size of the squared landing field 10*10 m
#define MAX_VALUE 1000                        //the maximal possible value for a line or landing field
#define MIN_VALUE 500                         //min value which is needed that a line or landing field is considered as good. 1000 is the Maximal value
#define SIZE_OF_ROW_BUFFER 1000               //size of the ring buffer which stores the line rows. Has to be great enough to store at least the hight of the landing fiels


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

typedef struct possibleLandingField{
  int value;
  int initValue;
  int hight;
  ros::Time time;
  int xPos;
  float speed;
}possibleLandingField_t;

typedef possibleLandingField_t* possibleLandingField_p;

static void handleLines(lineRow_p);
static void PCRowHandler(const sensor_msgs::PointCloud2ConstPtr);
static void updateVoltageLinePos(std_msgs::Int32);

#endif // E_L_F_D_H

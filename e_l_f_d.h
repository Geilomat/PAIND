#ifndef E_L_F_D_H
#define E_L_F_D_H

#define IS_SIMULATION 4                       //define if this is a simulation or runnning on real time
#define NUMBER_OF_POSSIBLE_LANDING_SIDES 10   //# of possible landing sides which shoulde be stored
#define LINE_PIECE_SIZE 1.0                  //size of one line piece [m]
#define DENSITY_PER_M 10                      //amount of points per meter which are needet for a line to be considered as possible
#define MAX_ACCEPTED_DIFFERENCE 0.5           //max accepted difference for one point [m]
#define MAX_ACCEPTED_SLOPE 0.33               //max accepted slope for the landing field +- 15 grde => +- 0.33m/m
#define VOLTAGE_LINE_SIZE 10                  //horizontal size of the voltage line
#define LANDING_FIELD_SIZE 10                 //size of the squared landing field 10*10 m


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
  ros::Time time;
  int xPos;
  float velocity;
}possibleLandingField_t;

typedef possibleLandingField_t* possibleLandingField_p;

static void handleLines(lineRow_p);
static void PCRowHandler(const sensor_msgs::PointCloud2ConstPtr);
static void updateVoltageLinePos(std_msgs::Int32);
static void handleLandingFields(pcl_filter::LandingField);




#endif // E_L_F_D_H

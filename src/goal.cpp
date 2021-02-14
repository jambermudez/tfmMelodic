#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <functional>
#include <iostream>
#include <cstdlib>
   
/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/

double PI=3.1415;
double xGoal=0.0;
double yGoal=0.0;
double xGoalAux=0.0;
double yGoalAux=0.0;
bool flag = false;


class Turtlebot {

    public: Turtlebot();

        /**
         * This function should command the robot to reach the goal
         * It should compute the commands to the robot by knowing the current position
         * and the goal position
         */
        bool command(double goal_x, double goal_y);
        
        private: ros::NodeHandle nh_;

        //Publisher and subscribers
        ros::Publisher vel_pub_;
        ros::Subscriber sub_laser;

        //Transform listerner to obtain the transform between the world frame (odom) and the robot frame (base_link)
        tf::TransformListener listener;
        
        //!Publish the command to the turtlebot
        void publish(double angular_vel, double linear_vel);
        
};

void laserCallBack(const sensor_msgs::LaserScan& msg){

  float size_readings;
    
  size_readings=msg.ranges.size();
  /*
  ROS_INFO("Num Lecturas (%f).", msg.ranges.size() );
  ROS_INFO("Rango minimo (%f).", msg.range_min );
  ROS_INFO("Rango maximo (%f).", msg.range_max );
  ROS_INFO("Angulo minimo (%f).", msg.angle_min );
  ROS_INFO("Angulo maximo (%f).", msg.angle_max );
  ROS_INFO("Angulo incre (%f).", msg.angle_increment );

    Num Lecturas (0.000000).
[ INFO] [1607112841.825648958, 410.346000000]: Rango minimo (0.120000).
[ INFO] [1607112841.825677521, 410.346000000]: Rango maximo (3.500000).
[ INFO] [1607112841.825699649, 410.346000000]: Angulo minimo (0.000000).
[ INFO] [1607112841.825717772, 410.346000000]: Angulo maximo (6.283190).
[ INFO] [1607112841.825753707, 410.346000000]: Angulo incre (0.017502).
*/

  //ROS_INFO("Ranges(%f).", msg.ranges[250]);


 std::ostringstream buff;
  //buff<<msg.ranges[250];

  // ( buff.str() == "inf"){
  //  ROS_INFO("Entra en el IF");
  //}

  
  if (msg.ranges[0]<=2){
    for (int i=0; i<=180; i++){
      buff<<msg.ranges[i];
      if ( buff.str() == "inf"){
        ROS_INFO("ENTRA EN laserCallBack %d", i);
      }else{
        ROS_INFO("ENTRA EN laserCallBack %.2f, %d", msg.ranges[i], i);
      }
    }

    flag = true;
  }else{
    flag = false;
  }

}


Turtlebot::Turtlebot() {
    //vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  sub_laser = nh_.subscribe("scan", 1000, laserCallBack);
  
}

int main(int argc, char** argv) {

  if(argc<3){
    std::cout << "Insuficient number of parameters" << std::endl;
    std::cout << "Usage: c1 Goal_coordinate_x Goal_coordinate_y" << std::endl;
    return 0;
  }

  xGoal=std::atof(argv[1]);
  yGoal=std::atof(argv[2]);

    ros::init(argc, argv, "robot_control");
    Turtlebot robot;
    ros::NodeHandle n;

    ros::Rate loop_rate(20);
   
   bool prueba = false;

  while (ros::ok() && prueba==false){

    if (flag){
      prueba = robot.command(xGoalAux,xGoalAux);
    }else{
      prueba = robot.command(xGoal,yGoal);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
 }
    loop_rate.sleep();

    return 0;
}


bool Turtlebot::command(double gx, double gy) {

  bool prueba = false;
    
    double linear_vel=0.0;
    double angular_vel=0.0;

    
    //Transform the goal to the local frame
    geometry_msgs::PointStamped goal;
    geometry_msgs::PointStamped base_goal;

    goal.header.frame_id = "odom";
    //we'll just use the most recent transform available for our simple example
    goal.header.stamp = ros::Time();
    //just an arbitrary point in space
    goal.point.x = gx;
    goal.point.y = gy;
    goal.point.z = 0.0;
    
    tf::StampedTransform transform;

    try{
        listener.transformPoint("base_footprint", goal, base_goal);
         ROS_INFO("goal: (%.2f, %.2f. %.2f) -----> base_goal: (%.2f, %.2f, %.2f) at time %.2f",
            goal.point.x, goal.point.y, goal.point.z,
            base_goal.point.x, base_goal.point.y, base_goal.point.z, base_goal.header.stamp.toSec());
    }catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        return prueba;
    }

    double distGoalBase = sqrt(pow(base_goal.point.x,2) + pow(base_goal.point.y,2));
    double angulo = atan2(base_goal.point.y, base_goal.point.x); //*180/PI;s

    if (abs(angulo)>0.2){//orientar 10
      angular_vel=0.2;
      if(abs(angulo)<0.3){ //20
        angular_vel=0.1;
      }

    }else{
      if (abs(angulo)<=0.2 && abs(angulo)>=0.03){
        angular_vel=0.02;
      }

      if (distGoalBase<=0.05){
        linear_vel=0.0;
        angular_vel=0.0;
        prueba = true;
      }else{

        if (distGoalBase>2){
          if (abs(angulo)<=0.05 && abs(angulo)>=0){
            linear_vel=0.4;
          }else{
            linear_vel=0.3;
          }

        }else{
          linear_vel=distGoalBase*0.1;
        }
      }  
    }

    if (angulo<0){
      angular_vel=angular_vel*(-1);
    }

    publish(angular_vel,linear_vel);

    return prueba;
}

//Publish the control values to the turtlebot
void Turtlebot::publish(double angular, double linear) {      
  geometry_msgs::Twist vel_msg;
      
  vel_msg.linear.x = linear;
  vel_msg.angular.z = angular;
    
  vel_pub_.publish(vel_msg);
  return;
}
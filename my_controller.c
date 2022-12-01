/*
 * File:          PID_Control.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <stdio.h>
#include <math.h>
#include <webots/distance_sensor.h>
#include <stdlib.h>
#include <webots/touch_sensor.h>


#define TIME_STEP 16
//turning funciton
void turn( WbDeviceTag Moto1, WbDeviceTag Moto2, float pid){

  int speed = 1+ 1*fabs(pid)/100 ;
 
      

    
 if(pid < 0){
     
  wb_motor_set_velocity(Moto1, -1*speed);
  wb_motor_set_velocity(Moto2, speed);
  
  
  }
  else if (pid > 0){
  wb_motor_set_velocity(Moto1, speed);
  wb_motor_set_velocity(Moto2, -1*speed);
  }
  }
  
 //vehicle stop function 
void stop(WbDeviceTag Motor1,WbDeviceTag Motor2){

  wb_motor_set_velocity(Motor1, 0);
  wb_motor_set_velocity(Motor2, 0);
  }

//move forward funciton
void forward(WbDeviceTag Motor1 ,WbDeviceTag Motor2, float pid){

float mod = 4*pid/100;
float speed = 3 ;
  if (pid < 0){
  wb_motor_set_velocity(Motor1, speed- fabs(mod));
  wb_motor_set_velocity(Motor2, speed+ fabs(mod));
  }
  
  if (pid > 0){
  wb_motor_set_velocity(Motor1, speed + fabs(mod));
  wb_motor_set_velocity(Motor2, speed - fabs(mod));
  }
  }

//delay function
void delay(int milliseconds) { 
    // Storing start time 
    double start_time = wb_robot_get_time()*1000; 
    // looping till required time is not achieved 
    while (wb_robot_get_time()*1000 < start_time + milliseconds){
    wb_robot_step(TIME_STEP); 
    }
}



/*
 * You may want to add macros here.
 */


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
 //create device tag
  WbDeviceTag Motor1 = wb_robot_get_device("Motor1");
  WbDeviceTag Motor2 = wb_robot_get_device("Motor2");
  WbDeviceTag bmotor = wb_robot_get_device("base motor");
  WbDeviceTag amotor = wb_robot_get_device("arm_motor");
  WbDeviceTag gbmotor = wb_robot_get_device("gbase_motor");// gripper base motor
  WbDeviceTag gmotor1 = wb_robot_get_device("gmotor1"); //gripper motor
  WbDeviceTag gmotor2 = wb_robot_get_device("gmotor2");// gripper motor
  
  //initialise motor position and speed
  wb_motor_set_position(Motor1, INFINITY);
  wb_motor_set_position(Motor2, INFINITY);
  wb_motor_set_position(bmotor, -1);
  wb_motor_set_position(amotor, 0);
  wb_motor_set_position(gbmotor, 0);
  wb_motor_set_position(gmotor1, 0);
  wb_motor_set_position(gmotor2, 0);
  
  wb_motor_set_velocity(Motor1, 0);
  wb_motor_set_velocity(Motor2, 0);
  wb_motor_set_velocity(bmotor, 0);
  wb_motor_set_velocity(amotor, 0);
  wb_motor_set_velocity(gbmotor, 1);
  wb_motor_set_velocity(gmotor1, 1);
  wb_motor_set_velocity(gmotor2, 1);
  
  
  
   //create sensor tag
   WbDeviceTag ds = wb_robot_get_device("DS"); //front sensor
   wb_distance_sensor_enable(ds, TIME_STEP);
   
    WbDeviceTag sds = wb_robot_get_device("sDS"); // side sensor
   wb_distance_sensor_enable(sds, TIME_STEP);
   
   //initialise touch sensor
  WbDeviceTag loadcell = wb_robot_get_device("loadcell");
  wb_touch_sensor_enable(loadcell, TIME_STEP);
   
  // initialise imu
  WbDeviceTag imu = wb_robot_get_device ("imu");
  wb_inertial_unit_enable( imu, TIME_STEP);
  

  //direction variables
  int modify = 1;
  int Max = 3; // 
  int left_turn = modify + Max; // a dummy variable to make decision between left and right 
  //int right_turn = base - modify;
  bool left = left_turn > Max; // instead of using numbers to decide, letters are used 
  bool right = left_turn < Max;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   //PID Control
   int kp = 3.5;
   int ki = 2;
   int kd = 0.8;
   
  float error_i = 0;
  float intgl = 0;
  
  
  
  float offset = -180; 
  float setpoint = 0;
  
  int stage = 0;
  
  // drop off zone storing
  float A=0; //D1
  float B=0;//D2
  float C=0; //D3
  float D=0;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
      float yaw  = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
      float dis = wb_distance_sensor_get_value(ds);
      float sdis = wb_distance_sensor_get_value(sds);
      float raw = wb_touch_sensor_get_value(loadcell);
      
    /* Process sensor data here */
    
    float mass = (0.0102*raw -1)*1000; // convert raw value into mass
    printf(" mass : %.2f  grams \n", mass);
    
    yaw = (yaw*180/M_PI) + offset;
    float distance = 0.6524*pow(dis,(-1.09));
    printf("distance : %f m\n ", distance);
    
    float sdistance = 0.521*pow( sdis, -1.967);
    printf("side distance : %f m\n ", sdistance);
    printf("A B C : %f / %f  /  %f  \n ", A,B,C);
    
    if (yaw > 180){
    yaw -=360;
    }
    else if (yaw < -180){
    yaw += 360;
    }
  
   
 //PID Calculations
  
  double error_n = setpoint - yaw;
  double dfntl  = (error_n - error_i)*1000/((TIME_STEP));
  intgl += error_n*(TIME_STEP/1000);
  
  //if (error_n * error_i <=0){
  //intgl = 0;
  //}
  
  double pid = (kp*error_n) + (ki*intgl) +(kd*dfntl);
  //error_i = error_n;
  
   if(fabs(pid) > 100){
   pid = pid/fabs(pid)*100;
   } 
  
  printf(" Heading : %f  error: %f  Differential : %f   Integral : %f   PID Output :  %f \n ", yaw, error_n, dfntl, intgl, pid);
  
  
   if (yaw >180){
  yaw -= 360;
  }
  else if (yaw <-180){
  yaw +=360;
  }
  /*
  turn(Motor1,Motor2,pid);
 
  if (pid < 0.5 ){
    if (distance > 0.3){
      forward(Motor1, Motor2, pid);
  }
    else{
    offset = offset + 90;
    }
    }
  else{
  turn(Motor1,Motor2,pid);
  }*/
  
  switch(stage){
    case 0 :
      if (pid >0.1 || pid < -0.1){
      turn(Motor1,Motor2,pid);
      }
      else{
      stop(Motor1,Motor2);
      stage = 1 ;
      }
      break;
    case 1 :
      if (distance >0.27){
        forward(Motor1,Motor2,pid);
        }
      else if (distance <= 0.27){
      stage = 2;
      offset = -90;
      }
      break;
    case 2 :
      if (pid > 0.1 || pid < -0.1){
        turn(Motor1,Motor2,pid);
      }
      else {
      stop(Motor1,Motor2);
      stage = 3;
      }
      break;
    case 3 :
      if (distance > 0.9){
      forward (Motor1,Motor2,pid);
        if (sdistance <= 1.1 && A<=0){
        A = distance;
        }
        else if ( sdistance <= 1.1 &&  A - distance >0.4 && B==0){
        B = distance;
        }
        else if ( sdistance < 1.1 && B - distance >0.4){
        C= distance;
        }
      }
      else if (distance <= 0.9){
      
      stop(Motor1,Motor2);
      delay(500);
      wb_motor_set_position(bmotor, -5);
      delay(2000);
      wb_motor_set_position(amotor, 2);
      delay(200);
      wb_motor_set_position(gbmotor, 0);
      delay(200);
      wb_motor_set_position(gmotor1, -0.14);
      wb_motor_set_position(gmotor2, 0.14);
      delay(200);
      wb_motor_set_position(gmotor1, 0);
      wb_motor_set_position(gmotor2, 0);
      delay(200);
      wb_motor_set_position(amotor, 2);
      delay(200);
      wb_motor_set_position(bmotor, -5);
      delay(200);
      stage = 9;
      //offset = 0;
      }
      break;
    //===========================================
    case 4 :
      if (pid >0.1 || pid < -0.1){
      turn(Motor1,Motor2,pid);
      }
      else{
      stop(Motor1,Motor2);
      stage = 5 ;
      }
      break;
    case 5 :
      if (distance >0.3){
        forward(Motor1,Motor2,pid);
        }
      else if (distance <= 0.3){
      stage = 6;
      offset = 90;
      }
      break;
    case 6 :
      if (pid > 0.1 || pid < -0.1){
        turn(Motor1,Motor2,pid);
      }
      else {
      stop(Motor1,Motor2);
      stage = 12;
      if (mass < 32 && mass > 9){
        D = 2-A;
        }
      else if (mass > 43 && mass < 72){
        D = 2 - B ;
        }
      else if (mass > 78 && mass < 97){
        D = 2-B;
        }
      else {
      D = 0.3;
      }  
      }
      break;
      
   case 12 :
      if (distance > D){
      forward (Motor1,Motor2,pid);
      }
      else if (distance <= D){
      stop(Motor1,Motor2);
      delay(2000);
      //manipulate
      delay(500);
      wb_motor_set_position(bmotor, -0.1);
      delay(2000);
      wb_motor_set_position(amotor, 0.12);
      delay(200);
      wb_motor_set_position(gbmotor, 0);
      delay(200);
      wb_motor_set_position(gmotor1, 0);
      wb_motor_set_position(gmotor2, 0.0);
      delay(200);
      wb_motor_set_position(gmotor1, -0.14);
      wb_motor_set_position(gmotor2, 0.14);
      delay(200);
      wb_motor_set_position(amotor, 1.12);
      delay(200);
      wb_motor_set_position(bmotor, 1.1);
      delay(200);
      stage = 7;
      
      }
      break;
      
    case 7 :
      if (distance > 0.3){
      forward (Motor1,Motor2,pid);
      }
      else if (distance <= 0.3){
      stage = 8;
      offset = -180;
      }
      break;
    case 8 :
      if (pid > 0.1 || pid < -0.1){
        turn(Motor1,Motor2,pid);
      }
      else {
      stop(Motor1,Motor2);
      stage = 1;
      }
      break;
      
    case 9 :
      
      //manipulate
      if (mass > 5){
      
       if (distance > 0.3){
      forward (Motor1,Motor2,pid);
        if (sdistance <= 1.1 && A<=0){
        A = distance;
        }
        else if ( sdistance <= 1.1 &&  A - distance >0.4 && B==0){
        B = distance;
        }
        else if ( sdistance < 1.1 && B - distance >0.4 && C==0){
        C= distance;
        }
        }
        else if (distance <= 0.3){
      
        stop(Motor1,Motor2);
        stage = 4;
        offset = 0;
        }
      
      }
      else{
      if (distance >0.3){
        forward(Motor1,Motor2,pid);
        }
      else if (distance <= 0.3){
      stage = 10;
      offset = 58;
      }
      }
      break;
    case 10 :
      if (pid > 0.1 || pid < -0.1){
        turn(Motor1,Motor2,pid);
      }
      else {
      stop(Motor1,Motor2);
      stage = 11;
      }
      break;
      
    case 11 :
      if (distance >0.3){
        forward(Motor1,Motor2,pid);
        }
      else if (distance <= 0.3){
      stage = 0;
      offset = -180;
      }
      break; 
  }
 

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */


  /* Enter your cleanup code here */
}
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}


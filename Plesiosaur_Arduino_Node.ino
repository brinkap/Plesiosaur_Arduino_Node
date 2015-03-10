/* 
 * send 'ard_cmd_lin_vals' command to arduino and have linear actuators respond and update sampled 'ard_real_lin_vals'
 */
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

// Servo Control Parameters**********************************
Servo Servo0,Servo3,Servo4,Servo5,Servo6;
const int Pin_S0 = 2;
const int Pin_M10 = 10;
const int Pin_M11 = 11;
const int Pin_M20 = 12;
const int Pin_M21 = 13;
const int Pin_S3 = 3;
const int Pin_S4 = 4;
const int Pin_S5 = 5;
const int Pin_S6 = 6;
int analog_history_1[3] = {0,0,0};
int analog_history_2[3] = {0,0,0};
int errors[2];
int Motor_positions[2];
int Motor_commands[2] = {400,400};
int Servo_positions[5] = {90, 90, 90, 90, 90};
int Servo_commands[5] = {90, 90, 90, 90, 90};
int pwm_vals[2];
int max_rate_of_change[5] = {1, 2, 2, 2, 10};
float filterVal = .5;

void Execute_motor_cmds();

int median_filter(int value, int history[3]){
  
  history[2] = history[1];
  history[1] = history[0];
  history[0] = value;
  
  int middle;
  if ((history[0] <= history[1]) && (history[0] <= history[2])){
    middle = (history[1] <= history[2]) ? history[1] : history[2];
  }else if ((history[1] <= history[0]) && (history[1] <= history[2])){
    middle = (history[0] <= history[2]) ? history[0] : history[2];
  }else {
    middle = (history[0] <= history[1]) ? history[0] : history[1];
  }
 return middle;
}

// ROS parameters***********************************************

ros::NodeHandle  nh;

std_msgs::UInt16 val_1, val_2;
std_msgs::UInt16 cmd_0, cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6;
std_msgs::UInt16 ser_0, ser_3, ser_4, ser_5, ser_6;


ros::Publisher ard_real_ser_vals_0("ard_real_ser_vals_0", &ser_0);
ros::Publisher ard_real_lin_vals_1("ard_real_lin_vals_1", &val_1);
ros::Publisher ard_real_lin_vals_2("ard_real_lin_vals_2", &val_2);
ros::Publisher ard_real_ser_vals_3("ard_real_ser_vals_3", &ser_3);
ros::Publisher ard_real_ser_vals_4("ard_real_ser_vals_4", &ser_4);
ros::Publisher ard_real_ser_vals_5("ard_real_ser_vals_5", &ser_5);
ros::Publisher ard_real_ser_vals_6("ard_real_ser_vals_6", &ser_6);

ros::Publisher ard_cmd_echo_ser_vals_0("ard_cmd_echo_ser_vals_0", &cmd_0);
ros::Publisher ard_cmd_echo_lin_vals_1("ard_cmd_echo_lin_vals_1", &cmd_1);
ros::Publisher ard_cmd_echo_lin_vals_2("ard_cmd_echo_lin_vals_2", &cmd_2);
ros::Publisher ard_cmd_echo_ser_vals_3("ard_cmd_echo_ser_vals_3", &cmd_3);
ros::Publisher ard_cmd_echo_ser_vals_4("ard_cmd_echo_ser_vals_4", &cmd_4);
ros::Publisher ard_cmd_echo_ser_vals_5("ard_cmd_echo_ser_vals_5", &cmd_5);
ros::Publisher ard_cmd_echo_ser_vals_6("ard_cmd_echo_ser_vals_6", &cmd_6);

void ard_cmd_lin_vals_1Cb( const std_msgs::UInt16& linvalcmd_1){
  // command motor to linvalcmd
  if(linvalcmd_1.data < 160){
    Motor_commands[0] = 160;
  }else {
    Motor_commands[0] = linvalcmd_1.data;
  }

}

void ard_cmd_lin_vals_2Cb( const std_msgs::UInt16& linvalcmd_2){
  // command motor to linvalcmd
  Motor_commands[1] = linvalcmd_2.data;

}

void ard_cmd_ser_vals_0Cb( const std_msgs::UInt16& command){
  // command motor to linvalcmd
  Servo_commands[0] = command.data;
}
void ard_cmd_ser_vals_3Cb( const std_msgs::UInt16& command){
  // command motor to linvalcmd
  Servo_commands[1] = command.data;
}
void ard_cmd_ser_vals_4Cb( const std_msgs::UInt16& command){
  // command motor to linvalcmd
  Servo_commands[2] = command.data;
}
void ard_cmd_ser_vals_5Cb( const std_msgs::UInt16& command){
  // command motor to linvalcmd
  Servo_commands[3] = command.data;
}
void ard_cmd_ser_vals_6Cb( const std_msgs::UInt16& command){
  // command motor to linvalcmd
  Servo_commands[4] = command.data;
}

ros:: Subscriber<std_msgs::UInt16> sub0("ard_cmd_ser_vals_0",ard_cmd_ser_vals_0Cb);
ros:: Subscriber<std_msgs::UInt16> sub1("ard_cmd_lin_vals_1",ard_cmd_lin_vals_1Cb);
ros:: Subscriber<std_msgs::UInt16> sub2("ard_cmd_lin_vals_2",ard_cmd_lin_vals_2Cb);
ros:: Subscriber<std_msgs::UInt16> sub3("ard_cmd_ser_vals_3",ard_cmd_ser_vals_3Cb);
ros:: Subscriber<std_msgs::UInt16> sub4("ard_cmd_ser_vals_4",ard_cmd_ser_vals_4Cb);
ros:: Subscriber<std_msgs::UInt16> sub5("ard_cmd_ser_vals_5",ard_cmd_ser_vals_5Cb);
ros:: Subscriber<std_msgs::UInt16> sub6("ard_cmd_ser_vals_6",ard_cmd_ser_vals_6Cb);


void setup(){
  nh.initNode();
  nh.advertise(ard_real_lin_vals_1);
  nh.advertise(ard_real_lin_vals_2);
  nh.advertise(ard_real_ser_vals_0);
  nh.advertise(ard_real_ser_vals_3);
  nh.advertise(ard_real_ser_vals_4);
  nh.advertise(ard_real_ser_vals_5);
  nh.advertise(ard_real_ser_vals_6);
  nh.advertise(ard_cmd_echo_lin_vals_1);
  nh.advertise(ard_cmd_echo_lin_vals_2);
  nh.advertise(ard_cmd_echo_ser_vals_0);
  nh.advertise(ard_cmd_echo_ser_vals_3);
  nh.advertise(ard_cmd_echo_ser_vals_4);
  nh.advertise(ard_cmd_echo_ser_vals_5);
  nh.advertise(ard_cmd_echo_ser_vals_6);
  
  
  nh.subscribe(sub0);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);

  pinMode(Pin_M10, OUTPUT);
  pinMode(Pin_M11, OUTPUT);
  pinMode(Pin_M20, OUTPUT);
  pinMode(Pin_M21, OUTPUT);
  
  Servo0.attach(Pin_S0);
  Servo3.attach(Pin_S3);
  Servo4.attach(Pin_S4);
  Servo5.attach(Pin_S5);
  Servo6.attach(Pin_S6);
  
  digitalWrite(Pin_M10,LOW);
  digitalWrite(Pin_M11,LOW);
  digitalWrite(Pin_M20,LOW);
  digitalWrite(Pin_M21,LOW);
  
  Servo0.write(Servo_positions[0]);
  Servo3.write(Servo_positions[1]);
  Servo4.write(Servo_positions[2]);
  Servo5.write(Servo_positions[3]);
  Servo6.write(Servo_positions[4]);
  
  analog_history_1[0] = analogRead(6);
  analog_history_2[0] = analogRead(7);
}

void loop(){
  // update sensor feedback and ROS signals
  //Motor_positions[0] = analogRead(6);
  //Motor_positions[1] = analogRead(7);
  Motor_positions[0] = median_filter(analogRead(6), analog_history_1);
  Motor_positions[1] = median_filter(analogRead(7), analog_history_2);
  
  errors[0] = Motor_commands[0] - Motor_positions[0];
  errors[1] = Motor_commands[1] - Motor_positions[1];
  
  val_1.data = Motor_positions[0];
  val_2.data = Motor_positions[1];
  cmd_1.data = Motor_commands[0];
  cmd_2.data = Motor_commands[1];
  
  ser_0.data = Servo_positions[0];
  ser_3.data = Servo_positions[1];
  ser_4.data = Servo_positions[2];
  ser_5.data = Servo_positions[3];
  ser_6.data = Servo_positions[4];

  cmd_0.data = Servo_commands[0];
  cmd_3.data = Servo_commands[1];
  cmd_4.data = Servo_commands[2];
  cmd_5.data = Servo_commands[3];
  cmd_6.data = Servo_commands[4];
  
  
  // Publish all ROS signals
  ard_real_lin_vals_1.publish( &val_1 );
  ard_real_lin_vals_2.publish( &val_2 );
  ard_real_ser_vals_0.publish( &ser_0 );
  ard_real_ser_vals_3.publish( &ser_3 );
  ard_real_ser_vals_4.publish( &ser_4 );
  ard_real_ser_vals_5.publish( &ser_5 );
  ard_real_ser_vals_6.publish( &ser_6 );
  
  ard_cmd_echo_lin_vals_1.publish( &cmd_1);
  ard_cmd_echo_lin_vals_2.publish( &cmd_2);
  ard_cmd_echo_ser_vals_0.publish( &cmd_0);
  ard_cmd_echo_ser_vals_3.publish( &cmd_3);
  ard_cmd_echo_ser_vals_4.publish( &cmd_4);
  ard_cmd_echo_ser_vals_5.publish( &cmd_5);
  ard_cmd_echo_ser_vals_6.publish( &cmd_6);
 
  // M1: Calculate and command command signals and linearly decrease motor voltage close to set point
          
   pwm_vals[0] = (int)(255.0 * ((float)abs(errors[0])/50.0) );
   if(pwm_vals[0]>255){
     pwm_vals[0] = 255;
   }       
        
  if(abs(errors[0]) < 4){
    digitalWrite(Pin_M10, LOW);
    analogWrite(Pin_M11, 0);
    //pwm_1.data = 0;
  }else if(errors[0] > 0){
    digitalWrite(Pin_M10,LOW);
    analogWrite(Pin_M11,pwm_vals[0]);
    //pwm_1.data = -1*pwm_vals[0];
  }else{
    digitalWrite(Pin_M10,HIGH);
    analogWrite(Pin_M11,pwm_vals[0]);
    //pwm_1.data = pwm_vals[0];
  }
  
  // M2: Calculate and command command signals and linearly decrease motor voltage close to set point
          
    pwm_vals[1] = (int)(255.0 * ((float)abs(errors[1])/50.0) );
    if(pwm_vals[1] > 255){
      pwm_vals[1] = 255;
    } 
        
  if(abs(errors[1]) < 4){
    digitalWrite(Pin_M20, LOW);
    analogWrite(Pin_M21, 0);
    //pwm_2.data = 0;
  }else if(errors[1] > 0){
    digitalWrite(Pin_M20,HIGH);
    analogWrite(Pin_M21,pwm_vals[1]);
    //pwm_2.data = -1*pwm_vals[1];
  }else{
    digitalWrite(Pin_M20,LOW);
    analogWrite(Pin_M21,pwm_vals[1]);
    //pwm_2.data = pwm_vals[1];
  }      
  
  // Update servo positons and commands using max limit changes
  for(int iter = 0; iter < 5; iter++){
    if(Servo_positions[iter] != Servo_commands[iter]){
      int delta = Servo_commands[iter] - Servo_positions[iter];
      if(abs(delta) < max_rate_of_change[iter]){
        Servo_positions[iter] = Servo_commands[iter];
      }else if(delta < 0){
        Servo_positions[iter] = Servo_positions[iter] - max_rate_of_change[iter];
      }else{
        Servo_positions[iter] = Servo_positions[iter] + max_rate_of_change[iter];
      }
    }
  }
  
  Servo0.write(Servo_positions[0]);
  Servo3.write(Servo_positions[1]);
  Servo4.write(Servo_positions[2]);
  Servo5.write(Servo_positions[3]);
  Servo6.write(Servo_positions[4]);
  
  nh.spinOnce();
  delay(10);
}


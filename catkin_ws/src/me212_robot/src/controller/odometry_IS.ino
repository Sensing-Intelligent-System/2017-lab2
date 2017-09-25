#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define CPR 2970.0
#define RADIUS 0.042
#define WIDTH 0.235
int encoder_pos_L, encoder_pos_R;
int encoder_pre_L, encoder_pre_R;
float x, y, theta, d_theta;
float dis_per_tick = 2* PI* RADIUS / CPR;
float dis_L, dis_R;  
float pose[3];

ros::NodeHandle nh;
std_msgs::Float64MultiArray msg;
ros::Publisher odom("~odometry", &msg);

void setup() {
  // put your setup code here, to run once:
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  encoder_pos_L = 0;
  encoder_pos_R = 0;
  encoder_pre_L = 0;
  encoder_pre_R = 0;
  x = 0;
  y = 0;
  theta = 0;
  //Serial.begin(57600);
  attachInterrupt(0, Encoder_L, RISING);  //2
  attachInterrupt(1, Encoder_R, RISING);  //3 
  nh.initNode();
  nh.advertise(odom);
}

void loop() {
  // put your main code here, to run repeatedly:
  dis_L = dis_per_tick * (encoder_pre_L - encoder_pos_L);
  dis_R = dis_per_tick * (encoder_pre_R - encoder_pos_R);
  //////////////////////////////////////////////////////////////////
  // write your code

  // theta = ???
  // x = ??? 
  // y = ???

  ///////////////////////////////////////////////////////////////////
  msg.data_length = 3;
  pose[0] = x;
  pose[1] = y;
  pose[2] = theta;
  msg.data = pose;
  odom.publish(&msg);
  encoder_pos_L = encoder_pre_L;
  encoder_pos_R = encoder_pre_R;
  
  nh.spinOnce();
  delay(200);   // Update at 5 Hz
}
void Encoder_L()
{
  if(digitalRead(SPD_INT_L2) == HIGH)
    {
      encoder_pre_L -= 1;
      //Serial.println("Left -1");
    }
    
  else
  {
      encoder_pre_L += 1;
      //Serial.println("Left +1");
  }
}
void Encoder_R()
{
  if(digitalRead(SPD_INT_R2) == LOW)
    {
      encoder_pre_R -= 1;
      //Serial.println("Right -1");
    }
  else
    {
      encoder_pre_R += 1;
      //Serial.println("Right +1");
    }
}

#include <ros.h>
#include<Servo.h>
#include<Stepper.h>
#include <std_msgs/Byte.h>

ros::NodeHandle  nh;

int shoot = 0;
int reset_state = 0;

// bursless-motor parms
byte servoPin_R = 6;
byte servoPin_L = 3;
Servo servo_r;
Servo servo_l;
const int rightSp = 65;
const int leftSp = 95;

//stepper
int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void(* resetFunc) (void) = 0;//declare reset function at address 0

void cbshoot(const std_msgs::Byte& msg){
  shoot = msg.data;
}

void cbreset(const std_msgs::Byte& msg){
  reset_state = msg.data;
}

ros::Subscriber<std_msgs::Byte> sub("/shoot_cmd",cbshoot);
ros::Subscriber<std_msgs::Byte> reset("/shoot_reset",cbreset);

std_msgs::Byte shoot_pub;
ros::Publisher pub("shoot_back", &shoot_pub);

void setup()
{
  Serial.begin(57600);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(reset);
  nh.advertise(pub);
  
  myStepper.setSpeed(30);
//  servo_r.attach(servoPin_R);
//  servo_l.attach(servoPin_L);
}

void loop()
{
  if(shoot == 1){
    shoot_pub.data = 1;
//    servo_r.write(rightSp);
//    servo_l.write(leftSp);
    delay(1000);
    myStepper.setSpeed(30);
    myStepper.step(10);
    delay(500);
    myStepper.step(5);
    }else{
      shoot_pub.data= 0;
//      servo_r.write(10);
//      servo_l.write(10);
      myStepper.step(0);
      }
      
  shoot = 0;
  pub.publish(&shoot_pub);
  delay(1000);
  nh.spinOnce();

  if (reset_state == 1 ){
    resetFunc();
  }
  reset_state = 0;
}

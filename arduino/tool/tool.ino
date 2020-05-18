
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <Stepper.h>

// ROS
ros::NodeHandle  nh;

int action = 0;

void handle_msg( const std_msgs::Int8& msg){
    action = msg.data;
}

ros::Subscriber<std_msgs::Int8> sub_pc("arduino/action", handle_msg ); // Topic to gather the user's requests


std_msgs::Int8 state_msg;
ros::Publisher pub_ee_states("arduino/states", &state_msg); // Publisher of Arduino current state

std_msgs::Bool contact_msg;
ros::Publisher pub_contact_state("arduino/contact/state", &contact_msg); // Publisher of Arduino current state



// STEP MOTOR CONFIG
const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);


// MAGNET CONFIG
int IN1 = 3; // PWM signal
int IN2 = 4; // Difital signal
int ENB = 5; // PWM signal to avoid unnecessary overheating on the magnet

// CONTACT SENSOR CONFIG
int pushButton = 7;
int buttonState = 0;

void setup()
{
    // ROS Config
    nh.initNode();
    nh.advertise(pub_ee_states);
    nh.advertise(pub_contact_state);
    nh.subscribe(sub_pc);
    
    // Magnet
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Contact Sensor
    pinMode(pushButton, INPUT);
    
    // Set the speed to 5 rpm:
    myStepper.setSpeed(15);

}

void loop()
{
  state_msg.data = action; // Just publish what I have revieved
  // Contact sensor
  buttonState = digitalRead(pushButton);
  contact_msg.data = buttonState;
  pub_contact_state.publish( &contact_msg );
  
  pub_ee_states.publish( &state_msg );
  nh.spinOnce();
  //delay(500);
  
  if (action == 1 || action == -1){
    stepper(action);
    }
    else if (action == 2 || action == -2){
      magnet(action);
      }
      else{
        //pub_contact_state.publish( &contact_msg );
        //delay(1000); // Slow down the loop
       }
  delay(500);        
}

void magnet(int state)
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  
  if (state == 2){
    analogWrite(ENB,200); // Enable the magnet
    }
    else if (state == -2){
      analogWrite(ENB,0); // Disable the magnet
      action = 0;
      }
      else{
          analogWrite(ENB,0);
          action = 0; // Disable the magnet
          }
          
}


void stepper(int state){
    myStepper.step(state*stepsPerRevolution/2); // Clockwise until next position (like an action)
    action = 0; // Reset the value
    
  }

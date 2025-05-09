/*******************************************************************************\
| Creator: Johnel De Guzman                             Date Created: 4/19/2025 |
|                                                                               |
| Description:  Arduino script to subscribe to ROS topic for workstation        | 
|               commands specifying the conveyor belt to be turned ON or OFF.   |            
|               Subsquently, the status of the output pin signalling the relay  |
|               is published for confirmation that the command was carried out. |
|                                                                               |
\******************************************************************************/
 
// History:

// 04/26/2025 - Added Publishing functionality for Keyes IR Obstacle sensor & 
//		  the Sharp distance sensor.	  
// 04/27/2025 - Revised Keyes sensor for converting default output to boolean
//      for publishing to ROStopic.


// NOTES:
// 1. Need rosserial, rosserial-arduino, rosserial-python installed in workspace
// 2. May need to run the following in ubuntu terminal: sudo 666 <insert port address>
//    for communication between workstation and Arduino board (sets permission).
// 3. Make sure serial monitor is CLOSED if not getting connection when running the 
//    rosserial command below.
//    "rosrun rosserial_python serial_node.py <port address>"

#include <ros.h>
#include <std_msgs/Bool.h>	// For belt command
#include <std_msgs/Int32.h>	// For object detection (Keyes)
#include <std_msgs/Float32.h>	// For distance measurement (Sharp) Assess suitability as it works ~20cm-150cm

// Node handle setup
ros::NodeHandle nh;

// Messages to be sent out
std_msgs::Bool outputMessage1;
std_msgs::Bool outputMessage2;
std_msgs::Float32 outputMessage3;	// Perhaps change to int datatype for better precision

// The Arduino publisher to the topics that will be subscribed by ROS, matching the topicName in ROS subscriber
ros::Publisher pub_belt("stateCommanded", &outputMessage1);
ros::Publisher pub_keyes("objectDetected", &outputMessage2);
ros::Publisher pub_sharp("distMeasured", &outputMessage3);

// Initialize variable for Keyes Sensor
int enc_sens;
bool objDet;

// Initialize variable for Sharp sensor
int dist_volt;

// Callback function to run when a message is received from ROS
void callBackFunction(const std_msgs::Bool &inputMessage1) {
  if (inputMessage1.data) {
    digitalWrite(13, LOW); // Turn the Belt ON
    //delay(2500);
    outputMessage1.data=true;
  }
    else {
      digitalWrite(13, HIGH);  // Turn the Belt OFF
    //delay(2500);
      outputMessage1.data=false;
    }
    
  pub_belt.publish(&outputMessage1);	// Publish arduino changed belt command
  
}

// Subscriber to the topic that will be published by ROS, matching the topicName in ROS publisher
ros::Subscriber<std_msgs::Bool> sub("beltCommand", callBackFunction); // Keep below callbackFunction

void setup() {
  nh.initNode();		// Initiate Arduino node
  
  nh.advertise(pub_belt);	// Publisher for beltcommand
  nh.advertise(pub_keyes);	// Publisher for Keyes sensor
  nh.advertise(pub_sharp);	// Publisher for Sharp sensor
  
  nh.subscribe(sub);		// Suscriber to ROS topic for beltCommand
  
  pinMode(13, OUTPUT);    	// Initialize the digital pin as an output.
  digitalWrite(13, HIGH); 	// Start with belt turned off
  
  pinMode(7, INPUT);		// Initialize pin 7 for Keyes Sensor Input
  				// Note: Output high (1) by default for no object.
  
  // Serial.begin(57600);    // For testing Arduino separately
}
 
void loop() {
  nh.spinOnce();
  
  // For Keyes IR sensor
  enc_sens = digitalRead(7);	// Check what the Keyes sensor is detecting.
  if(enc_sens == 0){          // Sensor outputs 0 for object Detected
    objDet = true; 
    delay(1325);           // Update here into boolean to be published to topic
  }	else{
    objDet = false;
  }
  outputMessage2.data = objDet;
  pub_keyes.publish(&outputMessage2);
  
  // For Sharp distance sensor (might not be worth using)
  const int num_samples = 5;  // Number of samples for averaging to eliminate noise
  float sum = 0;
  for (int i = 0; i < num_samples; i++) {
    dist_volt = analogRead(A0);		// No need to initialize pin for Analog values, just wire correctly.
    float voltage = dist_volt * (5.0 / 1023.0);	// Convert to voltage (Arduino voltage/Read voltage)
    float distance_cm = 10650.08 * pow(voltage, -0.935); // GP2Y0A02YK0F formula (voltage to distance in cm)
    sum += distance_cm;
    delay(10);  // Short delay to avoid overloading the sensor
  }
  float distance_m = (sum / num_samples) / 100.0;	// Convert to meters

  // Assign to ROS message
  outputMessage3.data = distance_m;
  pub_sharp.publish(&outputMessage3);
  
  //Serial.println(objDet);   // For testing Arduino separately
  
  delay(20);	// Adjust this value depending on serial connection speed.
}

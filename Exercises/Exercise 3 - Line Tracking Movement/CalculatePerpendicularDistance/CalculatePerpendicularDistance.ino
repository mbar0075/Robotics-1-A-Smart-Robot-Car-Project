/* Including the necessary libraries and header files
  Wire.h: This library allows you to communicate with I2C / TWI devices.
  avr/wdt.h: This library allows you to use the watchdog timer to automatically reset the Arduino board.
  MPU6050.h: This header file contains the class definition for the MPU6050 class.
  MPU6050_getdata.h: This header file contains the class definition for the MPU6050_getdata class.
  DeviceDriverSet_xxx0.h: This header file contains the class definition for the DeviceDriverSet_xxx0 class.
*/
#include <Wire.h>
#include <avr/wdt.h>
#include "MPU6050.h"
#include "MPU6050_getdata.h"
#include "DeviceDriverSet_xxx0.h"

// Setting the MPU6050 Accelegryo component.
MPU6050_getdata AppMPU6050getdata;

// Creating a new servo object.
DeviceDriverSet_Servo AppServo;

// Declaring the variables.
float start_time = 0;
float current_angle = 0;

// Declaring variables for movement.
int x = 0;
int y = 0;
int angle = 0;

// Declaring Max Speed.
int MAX_SPEED = 255;
int current_speed = 100;

// Declaring the duration and distance variables.
long duration, distance;

// Declaring the distance threshold.
int distance_threshold = 20;

// Declaring Infrared Sensor Variables.
int left_sensor = 0;
int middle_sensor = 0;
int right_sensor = 0;

// Declaring variable to hold the perpendicular distance.
bool start = false;
bool stop = false;
float start_times = 0;
float end_times = 0;


void setup() {
  // Initialising the serial port for communication (9600 baud rate (the rate at which information is transferred in a communication channel)).
  Serial.begin(9600);

  /* Defining the pins to be used for the motors, tracking sensors, and ultrasonic sensor.
      PIN_Motor_STBY: Pin used to control the standby mode of the motors.
      PIN_Motor_PWMA: PWM pin for Motor A, used to control its speed.
      PIN_Motor_AIN_1: Input pin for Motor A to control its direction.
      PIN_Motor_PWMB: PWM pin for Motor B, used to control its speed.
      PIN_Motor_BIN_1: Input pin for Motor B to control its direction.
      PIN_ITR20001xxxL: Analog pin for the left tracking sensor.
      PIN_ITR20001xxxM: Analog pin for the middle tracking sensor.
      PIN_ITR20001xxxR: Analog pin for the right tracking sensor.
      TrackingDetection_S: Starting point of the tracking sensor array.
      TrackingDetection_E: End point of the tracking sensor array.
      echoPin: Pin for the ultrasonic sensor's echo.
      trigPin: Pin for the ultrasonic sensor's trigger.
  */
  #define PIN_Motor_STBY 3
  #define PIN_Motor_PWMA 5  
  #define PIN_Motor_AIN_1 7
  #define PIN_Motor_PWMB 6
  #define PIN_Motor_BIN_1 8
  #define PIN_ITR20001xxxL A2
  #define PIN_ITR20001xxxM A1
  #define PIN_ITR20001xxxR A0
  #define TrackingDetection_S 40
  #define TrackingDetection_E 270
  #define echoPin 12 
  #define trigPin 13

  /* Initialising the pin modes for all the individual component pins.
      INPUT: The pin is used to accept input external signals.
      OUTPUT: The pin is used to send output signals to external components.
  */
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Set the LED pin as an output.

  // Initialising the accelegyro component.
  AppMPU6050getdata.MPU6050_dveInit();

  // Calibrating the accelegyro sensor.
  AppMPU6050getdata.MPU6050_calibration();

  // Initialising the I2C (Inter-Integrated Circuit) communication bus.
  Wire.begin();

  // Retrieving the initial time.
  start_time = millis();

  // Initialising the servo to point directly forward.
  AppServo.DeviceDriverSet_Servo_Init(90);

  // Retrieving the current angle of the robot and storing it in the variable current_angle (passed by reference).
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&current_angle);

  // Calibrating the accelegyro sensor.
  AppMPU6050getdata.MPU6050_calibration();

  // Setting distance to a high value.
  distance = 1000;
}

void loop() {
  // Retrieving the current angle of the robot.
  float newAngle;
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&newAngle);

  // Retrieving the data from the infrared sensors.
  get_infrared_sensor_data();

  // Calculating the perpendicular distance of the robot from the obstacle.
  calculate_perpendicular_distance();

  if (!stop){
    // Moving the robot forward.
    fwd(get_speed(current_speed), get_speed(current_speed));
  }
  else{
    // Stopping the robot.
    stop_move();
  }

  // Delaying the loop by 100 milliseconds.
  delay(100);
}

void calculate_perpendicular_distance()
{
  /* Function to calculate the perpendicular distance of the robot from the obstacle.
  */
  // Checking if all the sensors are on the line.
  if (left_sensor > TrackingDetection_S && middle_sensor > TrackingDetection_S && right_sensor > TrackingDetection_S)
  {
    if (!start){
      // If the robot is on the line, set the start flag to true.
      start = true;

      // Retrieving the start time.
      start_times = millis();
    }
    else{
      // If the robot is not on the line, set the start flag to false.
      start = false;

      // Retrieving the end time.
      end_times = millis();

      // Calculating the time difference.
      float time_diff = end_times - start_times;

      // Calculating the perpendicular distance of the robot from the obstacle (distance = speed * time)
      float perpendicular_distance = time_diff * current_speed;

      // Changing the distance to cm (from mm).
      perpendicular_distance = perpendicular_distance / 1000;

      // Setting the stop flag to true.
      stop = true;

      // Printing the perpendicular distance to the serial monitor.
      Serial.print("Perpendicular Distance: ");
      Serial.println(perpendicular_distance);
    }
  }
}

bool is_obstacle(uint8_t distance_threshold)
{
  /* Function to check if there is an obstacle in front of the robot.
    Parameters: 
        distance: Distance of the robot from the obstacle.
  */
  // If the distance of the robot from the obstacle is less than 20 cm, return true.
  if (distance < distance_threshold)
  {
    return true;
  }

  // Otherwise, return false.
  return false;
}

int get_distance()
{
  /* Function to get the distance of the robot from the obstacle.
  */
  // Calculating the distance of the robot from the obstacle through the ultrasonic sensor.

  // Clearing the trigPin.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Setting the trigPin to HIGH state for 10 microseconds to send the ultrasonic pulse signal.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Calculating the duration of the ultrasonic pulse signal.
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance of the robot from the obstacle (speed of sound is 0.034 cm per microsecond and the ultrasonic sensor sends a pulse signal to the obstacle and back).
  distance = duration * 0.034 / 2; // Divide by 2 to get the distance to the obstacle (go and return).

  // Return the distance.
  return distance;
}

void get_infrared_sensor_data()
{
  /* Function to get the data from the infrared sensors.
    Variables:
      left_sensor: Data from the left tracking sensor.
      middle_sensor: Data from the middle tracking sensor.
      right_sensor: Data from the right tracking sensor.
  */
  // Reading the data from the left tracking sensor.
  left_sensor = analogRead(PIN_ITR20001xxxL);

  // Reading the data from the middle tracking sensor.
  middle_sensor = analogRead(PIN_ITR20001xxxM);

  // Reading the data from the right tracking sensor.
  right_sensor = analogRead(PIN_ITR20001xxxR);
}

int get_speed(uint8_t speed)
{
  /* Function to get the speed of the robot.
    Parameters: 
        speed: Speed of the robot.
  */
  // If the speed is greater than the maximum speed, set the speed to the maximum speed.
  if (speed > MAX_SPEED)
  {
    speed = MAX_SPEED;
  }

  // Return the speed.
  return speed;
}

void fwd(uint8_t right_motor_speed, uint8_t left_motor_speed)
{
  /* Function to move the robot forward.
   Parameters: 
      right_motor_speed: Speed for the right pair of motors.
      left_motor_speed: Speed for the left pair of motors.

    Digital signals:
      HIGH: represents the digital signal of 1.
      LOW: represents the digital signal of 0.
  */
  // Setting the direction of the motors.
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);

  // Setting the speed of the motors.
  analogWrite(PIN_Motor_PWMB, left_motor_speed);
  analogWrite(PIN_Motor_PWMA, right_motor_speed);

  // Enabling the motors.
  digitalWrite(PIN_Motor_STBY, HIGH);
}

void bwd(uint8_t right_motor_speed, uint8_t left_motor_speed)
{
  /* Function to move the robot backward.
    Parameters: 
        right_motor_speed: Speed for the right pair of motors.
        left_motor_speed: Speed for the left pair of motors.
  
      Digital signals:
        HIGH: represents the digital signal of 1.
        LOW: represents the digital signal of 0.
  */
  // Setting the direction of the motors.
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);

  // Setting the speed of the motors.
  analogWrite(PIN_Motor_PWMA, right_motor_speed);
  analogWrite(PIN_Motor_PWMB, left_motor_speed);

  // Enabling the motors.
  digitalWrite(PIN_Motor_STBY, HIGH);
}

void stop_move()
{
  /* Function to stop the robot from moving.
    Digital signals:
      HIGH: represents the digital signal of 1.
      LOW: represents the digital signal of 0.
  */
  // Stopping the motors by setting the speed to 0.
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);

  // Disabling the motors.
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);

  // Disabling the standby mode.
  digitalWrite(PIN_Motor_STBY, LOW);
}

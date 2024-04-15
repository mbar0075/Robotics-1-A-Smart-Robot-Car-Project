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
bool LS, MS, RS = false;


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
  #define TrackingDetection_S 250
  #define TrackingDetection_E 850
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

  // Get the infrared sensor data.
  get_infrared_sensor_data();

  if (LS || MS || RS)
  {
    // Line Tracking.
    line_tracking();
  }
  else
  {
    // Obstacle Avoidance.
    obstacle_avoidance();
  }

  // Delaying the loop by 100 milliseconds.
  delay(100);
}

void line_tracking()
{
  /* Function to track the line.
  */
  // Retrieving the data from the infrared sensors.
  get_infrared_sensor_data();

  // Printing sensor data.
  // Serial.print("Left Sensor: ");
  // Serial.print(LS);
  // Serial.print(" Analog Value: ");
  // Serial.print(left_sensor);
  // Serial.print(" Middle Sensor: ");
  // Serial.print(MS);
  // Serial.print(" Analog Value: ");
  // Serial.print(middle_sensor);
  // Serial.print(" Right Sensor: ");
  // Serial.println(RS);
  // Serial.print(" Analog Value: ");
  // Serial.println(right_sensor);

  // If the robot is on the line.
  if (LS && MS && RS)
  {
    // Move the robot forward.
    fwd(get_speed(current_speed), get_speed(current_speed));
  }
  // If the robot is on the left side of the line.
  else if (!LS && MS && RS)
  {
    // Move the robot to the right.
    fwd(get_speed(current_speed), get_speed(0));
  }
  // If the robot is on the right side of the line.
  else if (LS && MS && !RS)
  {
    // Move the robot to the left.
    fwd(get_speed(0), get_speed(current_speed));
  }
  // If the robot is on the left side of the line.
  else if (!LS && MS && !RS)
  {
    // Move the robot to the right.
    fwd(get_speed(current_speed), get_speed(0));
  }
  // If the robot is on the right side of the line.
  else if (LS && !MS && RS)
  {
    // Move the robot to the left.
    fwd(get_speed(0), get_speed(current_speed));
  }
  // If the robot is on the left side of the line.
  else if (!LS && !MS && RS)
  {
    // Move the robot to the right.
    fwd(get_speed(current_speed), get_speed(0));
  }
  // If the robot is on the right side of the line.
  else if (LS && !MS && !RS)
  {
    // Move the robot to the left.
    fwd(get_speed(0), get_speed(current_speed));
  }
  // If the robot is off the line.
  else if (!LS && !MS && !RS)
  {
    // Serial.println("Off the line");
    // Stop the robot from moving.
    stop_move();
  }
}

void scan_and_evade_obstacle()
{
  /* Function to scan the environment for obstacles and evade them.
  */
  // First Stopping the robot from moving.
  stop_move();

  /* Iterating through the servo positions to scan the environment for obstacles.
    Parameters: 
        i: Position angle of the servo.

    Iterating from 1 to 6 with a step size of 2.

    Iteration Angles:
        1: 30 degrees.
        3. 90 degrees.
        5. 150 degrees.
  
  */
  for (uint8_t i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
  {
    // Setting the servo to point in the direction of the obstacle.
    AppServo.DeviceDriverSet_Servo_control(30 * i /* Position angle of the servo */);

    // Retrieving the distance of the robot from the obstacle.
    distance = get_distance();

    // If there is no obstacle in front of the robot then stop the robot from moving.
    if (is_obstacle(distance_threshold))
    {
      stop_move();
      
      // If 5th scan, i.e., 150 degrees, then move the robot backward and turn left or right based on a random number.
      if (i == 5)
      {
        // Going backward.
        bwd(get_speed(current_speed), get_speed(current_speed));
        delay(500);

        // Based on a random number, move the robot either left or right.
        if (random(0, 2) == 0)
        {
          // Going right.
          fwd(get_speed(current_speed), get_speed(0));;
        }
        else
        {
          // Going left.
          fwd(get_speed(0), get_speed(current_speed));
        }
        break;
      }
    }
    else{
      // First going little backward.
      bwd(get_speed(current_speed), get_speed(current_speed));
      delay(500);

      /* Performing switch statement for Obstacle Avoidance.

        Parameters: 
            i: Position angle of the servo.

        Cases:
            1: Going Right.
            3: Going Forward.
            5: Going Left.
      */
      switch (i)
      {
        case 1:
          // Going Right.
          fwd(get_speed(current_speed), get_speed(0));
        case 3:
          // Going Forward.
          fwd(get_speed(current_speed), get_speed(current_speed));
        case 5:
          // Going Left.
          fwd(get_speed(0), get_speed(current_speed));
      }

      // Delay for 50 milliseconds.
      delay(50);
      break;
    }
  }

  // Setting the servo to point directly forward.
  AppServo.DeviceDriverSet_Servo_control(90);
}

void obstacle_avoidance()
{
  /* Function to avoid obstacles.
    Parameters: 
        distance: Distance of the robot from the obstacle.
  */
  // Calculating the distance of the robot from the obstacle through the ultrasonic sensor.
  // Retrieving the distance of the robot from the obstacle.
  distance = get_distance();

  // If there is an obstacle in front of the robot.
  if (is_obstacle(distance_threshold))
  {
    // Scan the environment for obstacles.
    scan_and_evade_obstacle();
  }
  // Otherwise, move the robot forward.
  else
  {
    fwd(get_speed(current_speed), get_speed(current_speed));
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

  // Retrieving the detection status of the infrared sensors.
  LS = infrared_sensor_detection(left_sensor, TrackingDetection_S, TrackingDetection_E);
  MS = infrared_sensor_detection(middle_sensor, TrackingDetection_S, TrackingDetection_E);
  RS = infrared_sensor_detection(right_sensor, TrackingDetection_S, TrackingDetection_E);
}

bool infrared_sensor_detection(uint8_t sensor_value, uint8_t small_threshold, uint8_t large_threshold)
{
  /* Function to detect the infrared sensor.
    Parameters: 
        sensor_value: Value of the sensor.
        small_threshold: Small threshold value.
        large_threshold: Large threshold value.
  */
  // If the sensor value is within the small and large threshold, return true.
  if (sensor_value >= small_threshold && sensor_value <= large_threshold)
  {
    return true;
  }

  // Otherwise, return false.
  return false;
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

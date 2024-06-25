#include <Wire.h>
#include <MPU6050.h>
#include <AFMotor.h>
#include <Servo.h>

const int trigPin1 = A5; // Front ultrasonic sensor
const int echoPin1 = A4;
const int trigPin2 = A3; // Left Ultrasonic sensor
const int echoPin2 = A2;
const int trigPin3 = A1; // Right Ultrasonic sensor
const int echoPin3 = A0;
const int trigPin4 = A6; // Right Diagonal Ultrasonic sensor
const int echoPin4 = A7;
const int trigPin5 = A8; // Left Diagonal Ultrasonic sensor
const int echoPin5 = A9;

AF_DCMotor motor1(1);  
AF_DCMotor motor2(2);

Servo steeringServo; // Declare a Servo object

const int steeringPin = 9; // Pin connected to the steering servo

// PID constants
const float Kp = 1.0;
const float Ki = 0.1;
const float Kd = 0.01;

// Ellipse parameters
const float a = 12.5; // semi-major axis
const float b = 25.0; // semi-minor axis

// Timing variables
unsigned long previousMillis = 0;
const long interval = 100; // Control loop interval in milliseconds

// Error variables
float previous_error = 0;
float integral = 0;

int distanceFront; 
int distanceRight; 
int distanceLeft; 
int distanceRightDiagonal;
int distanceLeftDiagonal;

MPU6050 mpu;

// Odometry variables
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;
float posX = 0.0;
float posY = 0.0;
float theta = 0.0;

const float wheelRadius = 3.0; // in cm
const float wheelBase = 15.0;  // distance between wheels in cm
const int encoderPinA = 2; // Encoder pin for left wheel
const int encoderPinB = 3; // Encoder pin for right wheel

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin1, INPUT); 
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);
  pinMode(echoPin5, INPUT);
  Serial.begin(9600); 
  
  motor1.setSpeed(150);   
  motor2.setSpeed(150);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  steeringServo.attach(steeringPin); // Attach the servo to the specified pin

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoderRight, RISING);
}

void updateEncoderLeft() {
  encoderCountLeft++;
}

void updateEncoderRight() {
  encoderCountRight++;
}

// Function to get the current position
void getCurrentPosition(float &xc, float &yc) {
  long leftCount = encoderCountLeft;
  long rightCount = encoderCountRight;

  float leftDistance = leftCount * 2 * PI * wheelRadius / 360.0; // Convert counts to distance
  float rightDistance = rightCount * 2 * PI * wheelRadius / 360.0; // Convert counts to distance

  float distance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / wheelBase;

  theta += deltaTheta;
  posX += distance * cos(theta);
  posY += distance * sin(theta);

  xc = posX;
  yc = posY;
}

// Function to get the current heading
float getCurrentHeading() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float heading = atan2(ay, ax) * 180 / PI; // Calculate heading in degrees
  if (heading < 0) {
    heading += 360;
  }
  return heading;
}

// Function to set the steering angle
void setSteeringAngle(float angle) {
  // Map the angle to the servo range (usually 0 to 180 degrees)
  int servoAngle = map(angle, -45, 45, 0, 180);
  steeringServo.write(servoAngle); // Set the servo to the calculated angle
}

// Function to find the closest point on the ellipse
void closestPointOnEllipse(float xc, float yc, float &x_d, float &y_d, float &t_d) {
  t_d = atan2(b * yc, a * xc);
  x_d = a * cos(t_d);
  y_d = b * sin(t_d);
}

int ultrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int cm = duration * 0.034 / 2; // Calculate distance
  return cm;
}

void loop() {
  distanceFront = ultrasonicDistance(trigPin1, echoPin1);
  Serial.print("DistanceFront: ");
  Serial.println(distanceFront);
    
  distanceLeft = ultrasonicDistance(trigPin2, echoPin2);
  Serial.print("DistanceLeft: ");
  Serial.println(distanceLeft);
    
  distanceRight = ultrasonicDistance(trigPin3, echoPin3);   
  Serial.print("DistanceRight: ");
  Serial.println(distanceRight);

  distanceRightDiagonal = ultrasonicDistance(trigPin4, echoPin4);   
  Serial.print("DistanceRightDiagonal: ");
  Serial.println(distanceRightDiagonal);

  distanceLeftDiagonal = ultrasonicDistance(trigPin5, echoPin5);   
  Serial.print("DistanceLeftDiagonal: ");
  Serial.println(distanceLeftDiagonal);

  if(distanceFront > 25) {
    forward();
  } else if(distanceRightDiagonal > 27.95) {
    right();
  } else if (distanceLeftDiagonal > 27.95) {
    left();
  } else {
    backward();
    if (distanceLeft > 25) {
      left();
    } else if (distanceRight > 25) {
      right();
    }
  }
}

void backward() {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}

void forward() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}

void left() {
  adjustSteering();
}

void right() {
  adjustSteering();
}

void adjustSteering() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Get the current position and orientation
    float xc, yc;
    getCurrentPosition(xc, yc);
    float theta_c = getCurrentHeading();

    // Find the closest point on the ellipse
    float x_d, y_d, t_d;
    closestPointOnEllipse(xc, yc, x_d, y_d, t_d);

    // Desired heading at the closest point
    float theta_d = atan2(b * cos(t_d), -a * sin(t_d));

    // Calculate errors
    float cte = sqrt(pow(xc - x_d, 2) + pow(yc - y_d, 2));
    float heading_error = theta_d - theta_c;

    // PID calculations
    float P_term = Kp * cte;
    integral += cte * (interval / 1000.0);
    float I_term = Ki * integral;
    float derivative = (cte - previous_error) / (interval / 1000.0);
    float D_term = Kd * derivative;

    // Compute control output
    float control_output = P_term + I_term + D_term;

    // Apply control output (adjust steering angle)
    setSteeringAngle(control_output);

    // Update previous error
    previous_error = cte;
  }
}

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
/////////////////////////
#define YB_ServoPin      6
#define YA_ServoPin      5
#define XA_ServoPin      9
#define XB_ServoPin      10 //#TODO: Rewire from pin 11 to free up MOSI Pin 
////////////////////////
#define P_GAIN           0.2     
#define I_GAIN           0.02 
#define D_GAIN           0.09 
unsigned long TrackedTimes[4] = {0,0,0,0}; // time at the end of the last loop
#define TOTAL_FLIGHT_TIME 7000
#define CONTROLLER_UPDATE_FREQUENCY 300 //Hz
#define ESTIMATE_UPDATE_FREQUENCY 200 //Hz
#define LOGGING_FREQUENCY 300 //HzDegrees
///////////////////////////////
#define GYROSCOPE_SENSITIVITY 65.536    // Definitely forgot what this number means 
/////////////////////////
#define MAX_OFFSET      90
int MAX_ANGLE =         90 + MAX_OFFSET; // degrees
int MIN_ANGLE =         90 - MAX_OFFSET; // degrees
//Globals

class PIDController
{
  private:
    bool Clamped = false;
    int min_value;
    int max_value;
    bool sameSign(float a, float b){return (a / abs(a)) == (b / abs(b));}
    int CheckClamp(int a) {
      // Bound the input value between x_min and x_max. Also works in anti-windup
      int angle = constrain(a, MIN_ANGLE, MAX_ANGLE); // Angle Limit
      Clamped = not (angle == a);
      return angle;
    }
  public:
    int DEADZONE = 15;
    float IntegralTerm = 0;
    float DerivativeTerm = 0;
    float Error = 0;
    int Output = 0;
    float lastError = 0;
    float lastOutput = 0;
    int TargetValue;
    PIDController(int targ, int mini, int maxi) {
      TargetValue = targ;
      min_value = mini;
      max_value = maxi;
      }
    void set_Target(int value ) { // -180 :-: 180
      TargetValue = constrain(value, min_value, max_value); // Speed Limit
      }
    void Update(int MeasuredValue){
      // compute the error between the measurement and the desired value
      Error = -ShortestAngularPath(MeasuredValue, TargetValue);
      if (abs(Error) <= DEADZONE) { // Deadzone // Stop if close enough to prevent oscillations
        lastOutput = TargetValue;
      } else {
        DerivativeTerm = Error - lastError;
        // If the actuator is saturating ignore the integral term
        // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
        if (Clamped and sameSign(Output, IntegralTerm)) {
          IntegralTerm += 0;
        } else {
          IntegralTerm += Error;
        }
        // compute the control effort by multiplying the error by Kp
        Output = 90 + (Error * P_GAIN) + (IntegralTerm * I_GAIN) + (DerivativeTerm * D_GAIN);
        lastError = Error;

        // make sure the output value is bounded to 0 to 100 using the bound function defined below
        lastOutput = CheckClamp(lastOutput);
      }
    }
  };

Servo XA_Servo;
Servo XB_Servo;
Servo YA_Servo;
Servo YB_Servo;
// Initialize the IMU Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
File Logger;
bool Launched = false;

float AngleEstimates[3]; //

//Variables below don't need to be global but we expose them for debug purposes
float Accel[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float Gyro[3];          // Gyro Readings in deg/sec
int CheckClamp(int, char); 

void ApplyComplementaryFiltering(unsigned long delta_time) {
    float pitchAcc, rollAcc;               
    delta_time /= 1000.0f; //Convert to seconds 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    AngleEstimates[0] += ((float)Gyro[0] / GYROSCOPE_SENSITIVITY) * delta_time; // Angle around the X-axis
    AngleEstimates[1] -= ((float)Gyro[1] / GYROSCOPE_SENSITIVITY) * delta_time; // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(Accel[0]) + abs(Accel[1]) + abs(Accel[2]);
    if (forceMagnitudeApprox > .5 && forceMagnitudeApprox < 2)
    { 
    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)Accel[1], (float)Accel[2]) * 180 / PI;
        AngleEstimates[0] = 90 + (AngleEstimates[0] - 90) * 0.85 + pitchAcc * 0.15;
    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)Accel[0], (float)Accel[2]) * 180 / PI;
        AngleEstimates[1] = 90 + (AngleEstimates[1] - 90) * 0.85 + rollAcc * 0.15;
    }
  } 

void getMachineState() {
  // Convert DOF data to angles
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  Accel[0] = a.acceleration.x / 9.80665; // m/s^2 in g
  Accel[1] = a.acceleration.y / 9.80665; // m/s^2 in g
  Accel[2] = a.acceleration.z / 9.80665; // m/s^2 in g

  Gyro[0] = g.gyro.y / 1000;
  Gyro[1] = g.gyro.x / 1000;
  Gyro[2] = g.gyro.z / 1000;
  }
PIDController *X;
PIDController *Y;
void setup() {
  XA_Servo.attach(XA_ServoPin);
  XB_Servo.attach(XB_ServoPin);
  YA_Servo.attach(YA_ServoPin);
  YB_Servo.attach(YB_ServoPin);
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  Serial.println("LSM9DS1 Stabilized Rocket Demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin()) {
    FailureDance();
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  
  if (!SD.begin(4)) {
    Serial.println("SD Card initialization failed!");
    //while (1);
  }
  Serial.println("SD Card Initialized");
  Logger = SD.open("last_flight.txt", FILE_WRITE);
  
  Serial.end();
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  X = new PIDController(90,0,180);
  Y = new PIDController(90,0,180);
  SuccessDance();
  // helper to just set the default scaling we want
  setupSensor();
  // Add Comment
  getMachineState();
  }

void loop() {
  if(millis() - TrackedTimes[2] >= (1000 / ESTIMATE_UPDATE_FREQUENCY)) { // Enter Timed Loop 
    getMachineState();
    ApplyComplementaryFiltering(millis() - TrackedTimes[2]);
    TrackedTimes[2] = millis();
  }
  if (Launched){
    if(millis() - TrackedTimes[1] >= (1000 / CONTROLLER_UPDATE_FREQUENCY)) { // Enter Timed Loop 
      
      X->Update(AngleEstimates[0]);
      Y->Update(AngleEstimates[1]);
      set_X_Angle(X->Output);
      set_Y_Angle(Y->Output);
      TrackedTimes[1] = millis();
    }
    if(millis() - TrackedTimes[3] >= (1000 / LOGGING_FREQUENCY)) { // Enter Timed Loop 
      LogStateEstimates();
      TrackedTimes[3] = millis();
    }
    //LandingDetected();
  } else {
    TrackedTimes[0] = millis(); // Time since launch
    Launched = LaunchDetected();
  }
  }

///////// SETTERS ////////////////////
void set_X_Angle(int angle) {
  int inv_angle;
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  if (angle > 90) {
    inv_angle = angle - (2*abs(angle - 90));
  } else {
    inv_angle = angle + (2*abs(angle - 90));
  }
  XA_Servo.write(angle);
  XB_Servo.write(inv_angle);
  }

void set_Y_Angle(int angle) {
  int inv_angle;
  angle = constrain(angle, MIN_ANGLE,  MAX_ANGLE);
  if (angle > 90) {
      inv_angle = angle - (2*abs(angle - 90));
  } else {
      inv_angle = angle + (2*abs(angle - 90));
  }
  YA_Servo.write(angle);
  YB_Servo.write(inv_angle);
  }

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  /*/ 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  */
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  }
////////  USER POINTED FUNCTIONS ////////
void SuccessDance() {
  set_X_Angle(180);
  set_Y_Angle(180);
  delay(1000);
  set_X_Angle(0);
  set_Y_Angle(0);
  delay(1000);
  set_X_Angle(90);
  set_Y_Angle(90);
  delay(1000);
  }

void FailureDance() {
  YA_Servo.write(0);
  delay(300);
  XB_Servo.write(0);
  delay(300);
  YB_Servo.write(0);
  delay(300);
  XA_Servo.write(0);
  delay(1000);
  set_X_Angle(90);
  set_Y_Angle(90);
  }

void LogStateEstimates(){
  Logger.println("testing 1, 2, 3.");
  }
////////// IN-FLIGHT HELPER FUNCTIONS ////
bool LaunchDetected(){
  if (millis() > 3000){return true;}
  return false;
  }
bool LandingDetected(){
  if (millis() > 30000){return true;}
  return false;
  }
///////// HELPER FUNCTION ///////
float ShortestAngularPath(int angle, int goal) {
  // Minimum degree shifts in order to reach goal
  double raw_diff = angle > goal ? angle - goal : goal - angle;
  double dist = raw_diff > 180.0 ? 360.0 - raw_diff : raw_diff;
  if (goal <= 180) { // Theoretically fails for goal angles above 180 degrees
    if ((angle < goal) or (angle > (goal + 180))) {
      dist = dist;
    } else {
      dist = dist * -1;
    }
  }
  return dist;
  }

/*//////////IN DEVELOPMENT/////////////
void PlotXY(){
  Serial.print(0);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  Serial.print(AngleEstimates[0]);
  Serial.print(" ");
  Serial.println(AngleEstimates[1]);
  }
 */

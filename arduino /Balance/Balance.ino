#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>  // not used in this code but required!
#include <Adafruit_LSM9DS1.h>
/////////////////////////
#define YA_ServoPin      9
#define YB_ServoPin      6
#define XA_ServoPin      10
#define XB_ServoPin      11
////////////////////////
float P_GAIN =           0.2     ;
float I_GAIN =           0.02 ;
float D_GAIN =           0.09 ;
int   UPDATE_FREQUENCY = 10000; //Hz
#define DEADZONE         15 // Degrees

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536    
/////////////////////////
#define MAX_OFFSET      90
int MAX_ANGLE =         90 + MAX_OFFSET; // degrees
int MIN_ANGLE =         90 - MAX_OFFSET; // degrees
//Globals
unsigned long lastMilli = 0; // time at the end of the last loop

int current_x_speed_setting = 0;
int current_y_speed_setting = 0;

float goal_x_angle = 90;
float goal_y_angle = 90;
float current_steering_angle = 0;

struct config {          
  bool  Clamped = false;
  float IntegralTerm = 0;
  float DerivativeTerm = 0;
  float PID_Output = 0;
  float AngleError = 0;
  float lastAngleError = 0;
}; 

struct config x_config;
struct config y_config;

Servo XA_Servo;
Servo XB_Servo;
Servo YA_Servo;
Servo YB_Servo;
// Initialize the IMU Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

float RwEst[3];     //Rw estimated from combining RwAcc and RwGyro
float AngleEstimates[3]; //
unsigned long K_lastMicros;
unsigned long C_lastMicros;

//Variables below don't need to be global but we expose them for debug purposes
float RwAcc[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float GyroTemp[3];
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)

// Bound the input value between x_min and x_max. Also works in anti-windup
int CheckClamp(int a, char axis) {
  int angle = constrain(a, MIN_ANGLE, MAX_ANGLE); // Angle Limit
  if (axis == 'x') {
      x_config.Clamped = not (angle == a);
  } else if (axis == 'y') {
      y_config.Clamped = not (angle == a);
  }
  return angle;
}
 
void ApplyComplementaryFiltering() {
    float pitchAcc, rollAcc;               
    //compute interval since last sampling time
    static unsigned long newMicros = micros();
    unsigned long delta_time = (newMicros - C_lastMicros)/1000.0f; //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
    C_lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    AngleEstimates[0] += ((float)GyroTemp[0] / GYROSCOPE_SENSITIVITY) * delta_time; // Angle around the X-axis
    AngleEstimates[1] -= ((float)GyroTemp[1] / GYROSCOPE_SENSITIVITY) * delta_time; // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(RwAcc[0]) + abs(RwAcc[1]) + abs(RwAcc[2]);
    if (forceMagnitudeApprox > .5 && forceMagnitudeApprox < 2)
    { 
    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)RwAcc[1], (float)RwAcc[2]) * 180 / PI;
        AngleEstimates[0] = 90 + (AngleEstimates[0] - 90) * 0.85 + pitchAcc * 0.15;
    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)RwAcc[0], (float)RwAcc[2]) * 180 / PI;
        AngleEstimates[1] = 90 + (AngleEstimates[1] - 90) * 0.85 + rollAcc * 0.15;
    }
} 

void ApplyKalmanFiltering() {
  static int w;
  static float rad;
  static char signRzGyro;
  float wGyro = 20;

  //compute interval since last sampling time
  static unsigned long newMicros = micros();
  unsigned long delta_time = (newMicros - K_lastMicros)/1000.0f; //in seconds //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
  K_lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it

  //normalize vector (convert to a vector with same direction and with length 1)
  normalize3DVector(RwAcc);

  //evaluate RwGyro vector
  if (abs(RwEst[2]) < 0.1) {
    //Thetaz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
    //in this case skip the gyro data and just use previous estimate
    for (w = 0; w <= 2; w++) RwGyro[w] = RwEst[w];
  } else {
    //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
    for (w = 0; w <= 1; w++) {
      rad = GyroTemp[w];           // get current gyro rate in deg/ms
      rad *= delta_time;           // get angle change in deg
      Awz[w] = atan2(RwEst[w], RwEst[2]) * 180 / PI;  // get angle and convert to degrees
      Awz[w] += rad;                                 // get updated angle according to gyro movement
    }

    // estimate sign of RzGyro by looking in what qudrant the angle Axz is,
    // RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
    signRzGyro = GyroTemp[2]/abs(GyroTemp[2]); // (cos(Awz[0] * PI / 180) >= 0 ) ? 1 : -1;

    // reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
    for (w = 0; w <= 1; w++) {
      RwGyro[0] = sin(Awz[0] * PI / 180);
      RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
      RwGyro[1] = sin(Awz[1] * PI / 180);
      RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );
    }
    RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
  }

  // Combine Accelerometer and gyro readings
  for (w = 0; w <= 2; w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

  normalize3DVector(RwEst);
  
  // Store g estimates into angle estimates 
  for (w = 0; w <= 2; w++) AngleEstimates[w] = g2degree(RwEst[w]);
}

void UpdatePIDController_X() {
  // compute the error between the measurement and the desired value
  x_config.AngleError = -ShortestAngularPath(AngleEstimates[0], goal_x_angle);
  if (abs(x_config.AngleError) <= DEADZONE) { // Deadzone // Stop if close enough to prevent oscillations
    current_x_speed_setting = 90;
    set_X_Angle(current_x_speed_setting);
  } else {
    x_config.DerivativeTerm = x_config.AngleError - x_config.lastAngleError;
    // If the actuator is saturating ignore the integral term
    // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
    if (x_config.Clamped and sameSign(x_config.PID_Output, x_config.IntegralTerm)) {
      x_config.IntegralTerm += 0;
    } else {
      x_config.IntegralTerm += x_config.AngleError;
    }
    // compute the control effort by multiplying the error by Kp
    x_config.PID_Output = 90 + (x_config.AngleError * P_GAIN) + (x_config.IntegralTerm * I_GAIN) + (x_config.DerivativeTerm * D_GAIN);
    x_config.lastAngleError = x_config.AngleError;

    // make sure the output value is bounded to 0 to 100 using the bound function defined below
    current_x_speed_setting = CheckClamp(current_x_speed_setting,'x');
    set_X_Angle(current_x_speed_setting); // then write it to the LED pin to change control voltage to LED
  }
}

void UpdatePIDController_Y() {
  // compute the error between the measurement and the desired value
  y_config.AngleError = ShortestAngularPath(AngleEstimates[1], goal_y_angle); // Minimum degree shifts in order to reach goal
  if (abs(y_config.AngleError) <= DEADZONE) { // Deadzone
    current_y_speed_setting = 90;
    set_Y_Angle(current_y_speed_setting);
  } else {
    y_config.DerivativeTerm = y_config.AngleError - y_config.lastAngleError;
    // If the actuator is saturating ignore the integral term
    // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
    if (y_config.Clamped and sameSign(y_config.PID_Output,y_config.IntegralTerm)) {
      y_config.IntegralTerm += 0; 
    } else {
      y_config.IntegralTerm += y_config.AngleError;
    }
    // compute the control effort by multiplying the error by Kp
    y_config.PID_Output = 90 + (y_config.AngleError * P_GAIN) + (y_config.IntegralTerm * I_GAIN) + (y_config.DerivativeTerm * D_GAIN);
    y_config.lastAngleError = y_config.AngleError;
    // make sure the output value is bounded to 0 to 100 using the bound function defined below
    current_y_speed_setting = CheckClamp(y_config.PID_Output,'y');
    set_Y_Angle(current_y_speed_setting); // then write it to the LED pin to change control voltage to LED
  }
}

void UpdateLinearController_X() {
  // compute the error between the measurement and the desired value
    x_config.AngleError = -ShortestAngularPath(AngleEstimates[0], goal_x_angle);
    current_x_speed_setting = x_config.AngleError + AngleEstimates[0];
    set_X_Angle(current_x_speed_setting); // then write it to the LED pin to change control voltage to LED
}

void UpdateLinearController_Y() {
  // compute the error between the measurement and the desired value
    y_config.AngleError = -ShortestAngularPath(AngleEstimates[1], goal_y_angle);
    current_y_speed_setting = y_config.AngleError + AngleEstimates[1];
    set_Y_Angle(current_y_speed_setting); // then write it to the LED pin to change control voltage to LED
}

void getMachineState() {
  // Convert DOF data to angles
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  RwAcc[0] = a.acceleration.x / 9.80665; // m/s^2 in g
  RwAcc[1] = a.acceleration.y / 9.80665; // m/s^2 in g
  RwAcc[2] = a.acceleration.z / 9.80665; // m/s^2 in g

  GyroTemp[0] = g.gyro.y / 1000;
  GyroTemp[1] = g.gyro.x / 1000;
  GyroTemp[2] = g.gyro.z / 1000;
}

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
  SuccessDance();
  // helper to just set the default scaling we want
  setupSensor();
  // Add Comment
  getMachineState();
  for (int i = 0; i <= 2; i++) RwEst[i] = RwAcc[i]; //initialize with accelerometer readings
}

void loop() {
  getMachineState();
  //ApplyKalmanFiltering();
  ApplyComplementaryFiltering();
  UpdateLinearController_X();
  UpdateLinearController_Y();
  //UpdatePIDController_X();
  //UpdatePIDController_Y();
  PlotXY();
  delay(1000 / UPDATE_FREQUENCY); 
}

// Minimum degree shifts in order to reach goal
float ShortestAngularPath(int angle, int goal) {
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

void set_X_Angle(int angle) {
  int inv_angle;
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  if (angle > goal_x_angle) {
    inv_angle = angle - (2*abs(angle - goal_x_angle));
  } else {
    inv_angle = angle + (2*abs(angle - goal_x_angle));
  }
  XA_Servo.write(angle);
  XB_Servo.write(inv_angle);
}

void set_Y_Angle(int angle) {
  int inv_angle;
  angle = constrain(angle, MIN_ANGLE,  MAX_ANGLE);
  if (angle > goal_y_angle) {
      inv_angle = angle - (2*abs(angle - goal_y_angle));
  } else {
      inv_angle = angle + (2*abs(angle - goal_y_angle));
  }
  YA_Servo.write(angle);
  YB_Servo.write(inv_angle);
}

void set_X_Goal(int angle) { // -180 :-: 180
  angle = constrain(angle, -MAX_ANGLE, MAX_ANGLE); // Speed Limit
  goal_x_angle = angle;
}

void set_Y_Goal(int angle) { // -18 0 :-: 180
  angle = constrain(angle, -MAX_ANGLE, MAX_ANGLE); // Speed Limit
  goal_y_angle = angle;
}

void normalize3DVector(float* vector) {
  static float R;
  R = sqrt(squared(vector[0]) + squared(vector[1]) + squared(vector[2]));
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
}

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void SuccessDance() {
  set_X_Angle(180);
  set_Y_Angle(180);
  delay(1000);
  set_X_Angle(0);
  set_Y_Angle(0);
  delay(1000);
  set_X_Angle(90);
  set_Y_Angle(90);
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

void PlotXY(){
  Serial.print(0);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  Serial.print(AngleEstimates[0]);
  Serial.print(" ");
  Serial.println(AngleEstimates[1]);
}

float squared(float x) {return x * x;}

int g2degree(float g) {return constrain(((g + 1) / 2) * 180, 0, 180);} 

bool sameSign(float a, float b){return (a / abs(a)) == (b / abs(b));}

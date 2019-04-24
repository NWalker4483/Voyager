#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#define P_GAIN           0.3     
#define I_GAIN           0.07 
#define D_GAIN           0.2 

class PIDController {
  private:
    bool Clamped = false;
    int min_value;
    int max_value;
    float Error = 0;
    float IntegralTerm = 0;
    float DerivativeTerm = 0;
    
    bool sameSign(float a, float b){return (a / abs(a)) == (b / abs(b));}
    int CheckClamp(int a) {
      // Bound the input value between x_min and x_max. Also works in anti-windup
      int value = constrain(a, min_value, max_value); // Angle Limit
      Clamped = not (value == a);
      return value;
    }
  public:
    int DEADZONE = 0;
    
    int Output = 0;
    float lastError = 0;
    int TargetValue;
    float (*ErrorFunction)(int, int);
    PIDController(int targ, int mini, int maxi) {
      TargetValue = targ;
      min_value = mini;
      max_value = maxi;
      }
    PIDController(int targ, int mini, int maxi, float (*ptr)(int, int)) {
      TargetValue = targ;
      ErrorFunction = ptr;
      min_value = mini;
      max_value = maxi;
      }
    void set_Target(int value ) {
      TargetValue = constrain(value, min_value, max_value); // Speed Limit
      }
    void Update(int MeasuredValue){
      // compute the error between the measurement and the desired value
      Error = (*ErrorFunction)(MeasuredValue, TargetValue);
      if (abs(Error) <= DEADZONE) { // Deadzone // Stop if close enough to prevent oscillations
        Output = TargetValue;
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
        Output = (Error * P_GAIN) + (IntegralTerm * I_GAIN) + (DerivativeTerm * D_GAIN);
        lastError = Error;

        // make sure the output value is bounded to 0 to 100 using the bound function defined below
        Output = CheckClamp(Output);
      }
    }
  };

  class LinearController {
  private:
    int min_value;
    int max_value;
    float Error = 0;
  public:
    int DEADZONE = 0;
    int Output = 0;
    int TargetValue;
    float (*ErrorFunction)(int, int);
    LinearController(int targ, int mini, int maxi, float (*ptr)(int, int)) {
      TargetValue = targ;
      ErrorFunction = ptr;
      min_value = mini;
      max_value = maxi;
      }
    void set_Target(int value ) {
      TargetValue = constrain(value, min_value, max_value); // Speed Limit
      }
    void Update(int MeasuredValue){
      // compute the error between the measurement and the desired value
      Error = (*ErrorFunction)(MeasuredValue, TargetValue);
      if (abs(Error) <= DEADZONE) { // Deadzone // Stop if close enough to prevent oscillations
        Output = TargetValue;
      } else {
        // compute the control effort by multiplying the error by Kp
        Output = Error + Output;

        // make sure the output value is bounded to 0 to 100 using the bound function defined below
       // Output = CheckClamp(Output);
      }
    }
  };

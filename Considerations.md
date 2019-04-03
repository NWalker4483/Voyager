* Atmospheric Thinning 
* Altitude Dependant Gains
* What is the best fin shape for model rockets in: (General Rocket Notes)
* Allow for more aggressive angle correction past a certain threshold 
* Actively shifting the center of mass to stabilize during freefall i.e elastic bands released at ignition 

void UpdateLinearController(struct config state, char axis) {
  // compute the error between the measurement and the desired value
    if (axis=='x'){
      state.AngleError = -ShortestAngularPath(AngleEstimates[0], goal_x_angle);
    } else {
      state.AngleError = -ShortestAngularPath(AngleEstimates[1], goal_y_angle);
      }
    state.LastOutput = state.AngleError + AngleEstimates[0];
    if (axis=='x'){
      set_X_Angle(state.LastOutput); // then write it to the LED pin to change control voltage to LED
    } else {
      set_Y_Angle(state.LastOutput); // then write it to the LED pin to change control voltage to LED
      }
  }
/* Author : Nga Pham
 * Date: 4/15/2024
 * Course: ECEN345
 * Lab#: Lab#6
 * Desc: Provides a C program structure that emulates multi-tasking and
 * modularity for Behavior-based control with easy scalability
 */

#include "capi324v221.h"

// Behavior-Based Control Skeleton code.

// ---------------------- Defines:

#define DEG_90 150      /* Number of steps for a 90-degree (in place) turn. */
#define BASE_SPEED 100  // Starting speed
#define KP 0.3          // Proportional gain
#define KI 0.7          // Integral gain
#define KD 0.03         // Derivative gain

// Desc: This macro-function can be used to reset a motor-action structure
//       easily.  It is a helper macro-function.
#define __RESET_ACTION(motor_action) \
  do {                               \
    (motor_action).speed_L = 0;      \
    (motor_action).speed_R = 0;      \
    (motor_action).accel_L = 0;      \
    (motor_action).accel_R = 0;      \
    (motor_action).state = STARTUP;  \
  } while (0) /* end __RESET_ACTION() */

// Desc: This macro-fuction translates action to motion -- it is a helper
//       macro-function.
#define __MOTOR_ACTION(motor_action)                                    \
  do {                                                                  \
    STEPPER_set_accel2((motor_action).accel_L, (motor_action).accel_R); \
    STEPPER_runn((motor_action).speed_L, (motor_action).speed_R);       \
  } while (0) /* end __MOTOR_ACTION() */

// Desc: This macro-function is used to set the action, in a more natural
//       manner (as if it was a function).

// ---------------------- Type Declarations:

// Desc: The following custom enumerated type can be used to specify the
//       current state of the robot.  This parameter can be expanded upon
//       as complexity grows without intefering with the 'act()' function.
//		 It is a new type which can take the values of 0, 1, or 2 using
//		 the SYMBOLIC representations of STARTUP, EXPLORING, etc.
typedef enum ROBOT_STATE_TYPE {

  STARTUP = 0,  // 'Startup' state -- initial state upon RESET.
  EXPLORING,    // 'Exploring' state -- the robot is 'roaming around'.
  LINE_FOLLOW   // 'Line Follow' state -- the robot is following a black (dark) line.

} ROBOT_STATE;

// Desc: Structure encapsulates a 'motor' action. It contains parameters that
//       controls the motors 'down the line' with information depicting the
//       current state of the robot.  The 'state' variable is useful to
//       'print' information on the LCD based on the current 'state', for
//       example.
typedef struct MOTOR_ACTION_TYPE {
  ROBOT_STATE state;           // Holds the current STATE of the robot.
  signed short int speed_L;    // SPEED for LEFT  motor.
  signed short int speed_R;    // SPEED for RIGHT motor.
  unsigned short int accel_L;  // ACCELERATION for LEFT  motor.
  unsigned short int accel_R;  // ACCELERATION for RIGHT motor.

} MOTOR_ACTION;

// Desc: Structure encapsulates 'sensed' data.  Right now that only consists
//       of the state of the left & right IR sensors when queried.  You can
//       expand this structure and add additional custom fields as needed.
typedef struct SENSOR_DATA_TYPE {
  float left_line;   // Holds the storage for right line sensor reading.
  float right_line;  // Holds the storage for left line sensor reading.

} SENSOR_DATA;

// ---------------------- Globals:
volatile MOTOR_ACTION action;  // This variable holds parameters that determine
// the current action that is taking place.
// Here, a structure named "action" of type
// MOTOR_ACTION is declared.
float integral = 0;
float last_error = 0;

// ---------------------- Prototypes:
void LINE_sense(volatile SENSOR_DATA *pSensors, TIMER16 interval_ms);
void act(volatile MOTOR_ACTION *pAction);
void info_display(volatile MOTOR_ACTION *pAction);
BOOL compare_actions(volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b);
void LINE_Follow(volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors);

// ---------------------- Convenience Functions:
void info_display(volatile MOTOR_ACTION *pAction) {
  // NOTE:  We keep track of the 'previous' state to prevent the LCD
  //        display from being needlessly written, if there's  nothing
  //        new to display.  Otherwise, the screen will 'flicker' from
  //        too many writes.
  static ROBOT_STATE previous_state = STARTUP;

  if ((pAction->state != previous_state) || (pAction->state == STARTUP)) {
    LCD_clear();

    //  Display information based on the current 'ROBOT STATE'.
    switch (pAction->state) {
      case STARTUP:

        // Nofify program is about to start.
        LCD_printf("Starting...\n");

        break;

      case LINE_FOLLOW:
        LCD_printf("Following line...\n");

        break;

      default:

        LCD_printf("Unknown state!\n");

    }  // end switch()

    // Note the new state in effect.
    previous_state = pAction->state;

  }  // end if()

}  // end info_display()

// ----------------------------------------------------- //
BOOL compare_actions(volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b) {
  // NOTE:  The 'sole' purpose of this function is to
  //        compare the 'elements' of MOTOR_ACTION structures
  //        'a' and 'b' and see if 'any' differ.

  // Assume these actions are equal.
  BOOL rval = TRUE;

  if ((a->state != b->state) || (a->speed_L != b->speed_L) || (a->speed_R != b->speed_R) || (a->accel_L != b->accel_L) || (a->accel_R != b->accel_R))

    rval = FALSE;

  // Return comparison result.
  return rval;

}  // end compare_actions()

// ---------------------- Top-Level Behaviorals:
void LINE_sense(volatile SENSOR_DATA *pSensors, TIMER16 interval_ms) {
  // Sense must know if it's already sensing.
  //
  // NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
  //
  static BOOL timer_started = FALSE;

  // The 'sense' timer is used to control how often gathering sensor
  // data takes place.  The pace at which this happens needs to be
  // controlled.  So we're forced to use TIMER OBJECTS along with the
  // TIMER SERVICE.  It must be 'static' because the timer object must remain
  // 'alive' even when it is out of scope -- otherwise the program will crash.
  static TIMEROBJ sense_timer;

  // If this is the FIRST time that sense() is running, we need to start the
  // sense timer.  We do this ONLY ONCE!
  if (timer_started == FALSE) {
    // Start the 'sense timer' to tick on every 'interval_ms'.
    //
    // NOTE:  You can adjust the delay value to suit your needs.
    //
    TMRSRVC_new(&sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART, interval_ms);

    // Mark that the timer has already been started.
    timer_started = TRUE;

  }  // end if()

  // Otherwise, just do the usual thing and just 'sense'.
  else {
    // Only read the sensors when it is time to do so (e.g., every
    // 125ms).  Otherwise, do nothing.
    if (TIMER_ALARM(sense_timer)) {
      ADC_SAMPLE sample_line_left;
      ADC_SAMPLE sample_line_right;
      LCD_printf("read line sensor...\n");

      // NOTE: Just as a 'debugging' feature, let's also toggle the green LED
      //       to know that this is working for sure.  The LED will only
      //       toggle when 'it's time'.
      LED_toggle(LED_Green);

      // Read the left and right sensors, and store this
      // data in the 'SENSOR_DATA' structure.

      ADC_set_channel(ADC_CHAN3);
      sample_line_left = ADC_sample();
      pSensors->left_line = (sample_line_left * 5.0f) / 1024.0f;

      ADC_set_channel(ADC_CHAN4);
      sample_line_right = ADC_sample();
      pSensors->right_line = (sample_line_right * 5.0f) / 1024.0f;

      // Snooze the alarm so it can trigger again.
      TIMER_SNOOZE(sense_timer);

    }  // end if()

  }  // end else.

}  // end sense()
// -------------------------------------------- //
float PID(float setpoint, float measured_value) {
  float error = setpoint - measured_value;  
  integral += error;
  float derivative = error - last_error;
  last_error = error;
  return (KP * error) + (KI * integral) + (KD * derivative);
}  // end PID()
// -------------------------------------------- //
void LINE_Follow(volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors) {
  float left_sensor = pSensors->left_line;
	float right_sensor = pSensors->right_line;
  float position;
  LCD_clear();

  if ((right_sensor <= 4.3) || (right_sensor >= 4.7)) {
    position = pSensors->right_line;
  } else {
    position = pSensors->left_line;
  }

  pAction->state = LINE_FOLLOW;  // Change state to LINE_FOLLOW

  if (position == right_sensor) {
    float adjustment = PID(4.5, position);

    if ((position <= 4.3))

    {
      pAction->speed_L = BASE_SPEED - adjustment;
      pAction->speed_R = BASE_SPEED + adjustment;
      if (pAction->speed_L <= 0) {
        pAction->speed_L = 3;
      }
      LCD_printf_RC(3, 0, "shift left R\n");
    } else if (position >= 4.7) {
      pAction->speed_L = BASE_SPEED + adjustment;
      pAction->speed_R = BASE_SPEED - adjustment;
      if (pAction->speed_R <= 0) {
        pAction->speed_R = 3;
      }
      LCD_printf_RC(3, 0, "shift right\n");
    }
  }
  if (position == left_sensor) {
    if (position <= 4.3 || position >= 4.7) {
      float adjustment = PID(4.5, position);

      if ((position <= 4.3))

      {
        pAction->speed_L = BASE_SPEED + adjustment;
        pAction->speed_R = BASE_SPEED - adjustment;
        if (pAction->speed_R <= 0) {
          pAction->speed_R = 5;
        }
        LCD_printf_RC(3, 0, "shift right AL\n");
      }

      else if ((position >= 4.7))

      {
        pAction->speed_L = BASE_SPEED - adjustment;
        pAction->speed_R = BASE_SPEED + adjustment;
        if (pAction->speed_L <= 0) {
          pAction->speed_L = 5;
        }
        LCD_printf_RC(3, 0, "shift left AL\n");
      } else {
        pAction->speed_L = 50;
        pAction->speed_R = 50;
      }
    } else {
      LCD_printf_RC(3, 0, "straight\n");
      pAction->speed_L = 30;
      pAction->speed_R = 30;
    }
  }

  __MOTOR_ACTION(*pAction);

}  // end LINE_Follow()
// -------------------------------------------- //

void act(volatile MOTOR_ACTION *pAction) {
  // 'act()' always keeps track of the PREVIOUS action to determine
  // if a new action must be executed, and to execute such action ONLY
  // if any parameters in the 'MOTOR_ACTION' structure have changed.
  // This is necessary to prevent motor 'jitter'.
  static MOTOR_ACTION previous_action = {

      STARTUP, 0, 0, 0, 0

  };

  if (compare_actions(pAction, &previous_action) == FALSE) {
    // Perform the action.  Just call the 'free-running' version
    // of stepper move function and feed these same parameters.
    __MOTOR_ACTION(*pAction);

    // Save the previous action.
    previous_action = *pAction;

  }  // end if()

}  // end act()
// ---------------------- CBOT Main:
void CBOT_main(void) {
  volatile SENSOR_DATA sensor_data;

  // ** Open the needed modules.
  LED_open();      // Open the LED subsystem module.
  LCD_open();      // Open the LCD subsystem module.
  STEPPER_open();  // Open the STEPPER subsyste module.
  ADC_open();      // Open the ADC subsystem module.

  // Set the voltage reference (we want 5V reference).
  ADC_set_VREF(ADC_VREF_AVCC);

  // Reset the current motor action.
  __RESET_ACTION(action);

  // Wait 3 seconds or so.
  TMRSRVC_delay(TMR_SECS(3));

  // Clear the screen and enter the arbitration loop.
  LCD_clear();

  // Enter the 'arbitration' while() loop -- it is important that NONE
  // of the behavior functions listed in the arbitration loop BLOCK!
  // Behaviors are listed in increasing order of priority, with the last
  // behavior having the greatest priority (because it has the last 'say'
  // regarding motor action (or any action)).
  while (1) {
    // Sense must always happen first.
    // (IR sense happens every 125ms).
    LINE_sense(&sensor_data, 125);

    // Behaviors.
    LINE_Follow(&action, &sensor_data);

    // Perform the action of highest priority.
    act(&action);

    // Real-time display info, should happen last, if possible (
    // except for 'ballistic' behaviors).  Technically this is sort of
    // 'optional' as it does not constitute a 'behavior'.
    info_display(&action);

  }  // end while()

}  // end CBOT_main()

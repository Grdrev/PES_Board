#define PART1_DC_MOTOR 0
#define P1_M1 0 //open loop control
#define P1_M2 0 //Motor M2 Closed-Loop Velocity Control
#define P1_M3 0 //Motor M3 Closed-Loop Position Control

#define Part2_State_Machine 1

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end


// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    // PES-Board Pin Names
    /*
PB_PWM_M1
PB_PWM_M2
PB_PWM_M3

PB_ENC_A_M1
PB_ENC_B_M1
PB_ENC_A_M2
PB_ENC_B_M2
PB_ENC_A_M3
PB_ENC_B_M3

PB_ENABLE_DCMOTORS
*/
#if part1_DC_MOTOR
// create object to enable power electronics for the DC motors
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

#if P1_M1
// motor M1
FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1
/*
PWM input 0.0f → -12V is applied to the motor
PWM input 0.5f →  0V 
PWM input 1.0f →  12V
*/

#endif



#if P1_M2

const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                 // 6.0f V if you only use one battery pack


// motor M2
const float gear_ratio_M2 = 78.125f; // gear ratio
const float kn_M2 = 180.0f / 12.0f;  // motor constant [rpm/V]
// it is assumed that only one motor is available, therefore
// we use the pins from M2, so you can leave it connected to M2
DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);

// limit max. velocity to half physical possible velocity
motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);


// enable the motion planner for smooth movements
motor_M2.enableMotionPlanner();

#endif

#if P1_M3
// motor M3
// motor M3
const float gear_ratio_M3 = 78.125f; // gear ratio
const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
// it is assumed that only one motor is available, therefore
// we use the pins from M2, so you can leave it connected to M2
DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
// enable the motion planner for smooth movement
motor_M3.enableMotionPlanner();
// limit max. velocity to half physical possible velocity
motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);

#endif

#endif

#if Part2_State_Machine


// set up states for state machine
enum RobotState {
    INITIAL,
    SLEEP,
    FORWARD,
    BACKWARD,
    EMERGENCY
} robot_state = RobotState::INITIAL;

// mechanical button
DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                   // need to specify the mode for proper usage, see below
mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                   // is a defined potential


                                   // ultrasonic sensor
UltrasonicSensor us_sensor(PB_D3);
float us_distance_cm = 0.0f;

// create object to enable power electronics for the dc motors
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                 // 6.0f V if you only use one battery pack

// motor M3
const float gear_ratio_M3 = 78.125f; // gear ratio
const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
// it is assumed that only one motor is available, therefore
// we use the pins from M1, so you can leave it connected to M1
DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M3, kn_M3, voltage_max);
// enable the motion planner for smooth movement
motor_M3.enableMotionPlanner();
// limit max. acceleration to half of the default acceleration
motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);

#endif


// create DigitalOut object to command extra led, you need to add an additional resistor, e

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---
            // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1;

#if part1_DC_MOTOR
            #if P1_M1
            pwm_M1.write(0.75f); // apply 6V to the motor M1
            #endif

#if P1_M2
       
            // print to the serial terminal
//printf("Motor velocity: %f \n", motor_M2.getVelocity());

// limit max. velocity to half physical possible velocity
//motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);

//motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5f);
motor_M2.setRotation(3.0f); // set target rotation to 360 degrees

#endif

#if P1_M3
// rotate the motor 3 times
motor_M3.setRotation(3.0f);

// print to the serial terminal
printf("Motor position: %f \n", motor_M3.getRotation());

#endif
#endif

#if Part2_State_Machine
// state machine
switch (robot_state) {
        case RobotState::INITIAL: {
        // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
        enable_motors = 1;
        robot_state = RobotState::SLEEP;

        break;
    }
        case RobotState::SLEEP: {
        // wait for the signal from the user, so to run the process
        // that is triggered by clicking the mechanical button
        // then go to the FORWARD state
        if (mechanical_button.read())
            robot_state = RobotState::FORWARD;

        break;
    }
       case RobotState::FORWARD: {
        // press is moving forward until it reaches 1.5f rotations,
        // when reaching the value go to BACKWARD
        motor_M3.setRotation(1.5f); // setting this once would actually be enough
        // if the distance from the sensor is less than 4.5f cm,
        // we transition to the EMERGENCY state
        if (us_distance_cm < 4.5f)
            robot_state = RobotState::EMERGENCY;
        // switching condition is slightly smaller for robustness
        if (motor_M3.getRotation() > 1.49f)
            robot_state = RobotState::BACKWARD;

        break;
    }
        case RobotState::BACKWARD: {
        // move backwards to the initial position
        // and go to the SLEEP state if reached
        motor_M3.setRotation(0.0f);
        // switching condition is slightly bigger for robustness
        if (motor_M3.getRotation() < 0.01f)
            robot_state = RobotState::SLEEP;

        break;
    }
        case RobotState::EMERGENCY: {
        // disable the motion planner and
        // move to the initial position asap
        // then reset the system
        motor_M3.disableMotionPlanner();
        motor_M3.setRotation(0.0f);
        if (motor_M3.getRotation() < 0.01f)
            toggle_do_execute_main_fcn();

        break;
    }
    default: {

        break; // do nothing
    }
}
#endif

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---
                enable_motors = 0; // disable hardwaredriver DC motors

                
#if part1_DC_MOTOR
                #if P1_M1
                pwm_M1.write(0.5f); // apply 0V to the motor M1
                #endif

                #if P1_M2
                motor_M2.setRotation(0.0f); // set target rotation to 0 degrees
                #endif

                #if P1_M3
                motor_M3.setRotation(0.0f); // set target rotation to 0 degrees
                #endif

#endif

#if Part2_State_Machine

// reset variables and objects
led1 = 0;
enable_motors = 0;
us_distance_cm = 0.0f;
motor_M3.setMotionPlannerPosition(0.0f);
motor_M3.setMotionPlannerVelocity(0.0f);
motor_M3.enableMotionPlanner();
robot_state = RobotState::INITIAL;

#endif
                // reset variables and objects
                led1 = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---
#if part1_DC_MOTOR
        #if P1_M2
       
            // print to the serial terminal
printf("Motor velocity: %f \n", motor_M2.getVelocity());

#endif

#if P1_M3

// print to the serial terminal
printf("Motor position: %f \n", motor_M3.getRotation());

#endif

#endif

#if Part2_State_Machine

// read us sensor distance, only valid measurements will update us_distance_cm
const float us_distance_cm_candidate = us_sensor.read();
if (us_distance_cm_candidate > 0.0f)
    us_distance_cm = us_distance_cm_candidate;


// print to the serial terminal
printf("US Sensor in cm: %f, DC Motor Rotations: %f\n", us_distance_cm, motor_M3.getRotation());    
#endif

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
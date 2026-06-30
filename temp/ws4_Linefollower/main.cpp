#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "DCMotor.h"
//#include "LineFollower.h"
#include "LineFollower_Mod.h" //The file i will be modifying

#define USE_GEAR_RATIO_78 false    // set this to true use gear ratio 78.125, otherwise 100.00 is used

#define DEBUG


bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end



void printSensorBarData(const LineFollower &lineFollower);//For printing out the sensor bar data for debugging purposes, definition at the end


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

    // --- adding variables and objects and applying functions starts here ---

    printf("Alviness\n");

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
#if USE_GEAR_RATIO_78
    // https://www.pololu.com/product/3477/specs
    const float gear_ratio = 78.125f;
    const float kn = 180.0f / 12.0f;
#else
    // https://www.pololu.com/product/3490/specs
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;
#endif
    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

#if USE_GEAR_RATIO_78
    const float d_wheel = 0.035f;  // wheel diameter in meters
    const float b_wheel = 0.1518f; // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.118f; // distance from wheel axis to leds on sensor bar / array in meters
#else
    const float d_wheel = 0.0372f; // wheel diameter in meters
    const float b_wheel = 0.156f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.114f; // distance from wheel axis to leds on sensor bar / array in meters
#endif
    // line follower, tune max. vel rps to your needs
   LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());
    // nonlinear controller gains, tune to your needs
#if USE_GEAR_RATIO_78
    const float Kp = 1.0f * 2.0f;
    const float Kp_nl = 1.0f * 17.0f;
#else
    const float Kp = 1.2f * 2.0f;
    const float Kp_nl = 1.2f * 17.0f;
#endif
   lineFollower.setRotationalVelocityControllerGains(Kp, Kp_nl);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---

            // visual feedback that the main task is executed, setting this once would actually be enough
            enable_motors = 1;

            #ifdef DEBUG
            //printf("Main task is executed\n");
           // printf("Velocity M1: %0.2f rps, Velocity M2: %0.2f rps\n", lineFollower.getRightWheelVelocity(), lineFollower.getLeftWheelVelocity());
#endif

            // setpoints for the dc motors in rps
            motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
            motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        #ifdef DEBUG

        // print to the serial terminal
        printSensorBarData(lineFollower);

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


// Extra functions for debugging and testing purposes can be added here, e.g. to print out variables or object states to the console

void printSensorBarData(const LineFollower &lineFollower)
{
             // print to the serial terminal
printf("Averaged Bar Raw: |  %0.2f  | %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f | ", lineFollower.getAvgBit(0)
                                                                                                     , lineFollower.getAvgBit(1)
                                                                                                     , lineFollower.getAvgBit(2)
                                                                                                     , lineFollower.getAvgBit(3)
                                                                                                     , lineFollower.getAvgBit(4)
                                                                                                     , lineFollower.getAvgBit(5)
                                                                                                     , lineFollower.getAvgBit(6)
                                                                                                     , lineFollower.getAvgBit(7));
printf("Mean Left: %0.2f, Mean Center: %0.2f, Mean Right: %0.2f, Mean Outer: %0.2f \n", lineFollower.getMeanThreeAvgBitsLeft()
                                                                                      , lineFollower.getMeanFourAvgBitsCenter()
                                                                                      , lineFollower.getMeanThreeAvgBitsRight()
                                                                                      , lineFollower.getMeanFourAvgBitsOuter());
}
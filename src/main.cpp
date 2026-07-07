#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "IMU.h"

#include "Servo.h"

#define M_PIf 3.14159265358979323846f // pi

#define IMU_THREAD_DO_USE_MAG_FOR_MAHONY_UPDATE true

// IMU
#define PB_IMU_SDA PC_9
#define PB_IMU_SCL PA_8

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

       // servo
Servo servo_roll(PB_D0);
Servo servo_pitch(PB_D1);

// imu
ImuData imu_data;
IMU imu(PB_IMU_SDA, PB_IMU_SCL);
Eigen::Vector2f rp(0.0f, 0.0f);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
// modelcraft RS2 MG/BB
float servo_ang_min = 0.035f;
float servo_ang_max = 0.130f;

// servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
// servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

// angle limits of the servos
const float angle_range_min = -M_PIf / 2.0f;
const float angle_range_max =  M_PIf / 2.0f;

// angle to pulse width coefficients
const float normalised_angle_gain = 1.0f / M_PIf;
const float normalised_angle_offset = 0.5f;

// pulse width
static float roll_servo_width = 0.5f;
static float pitch_servo_width = 0.5f;

servo_roll.setPulseWidth(roll_servo_width);
servo_pitch.setPulseWidth(pitch_servo_width);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

            // --- code that runs when the blue button was pressed goes here ---


            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                led1 = 0;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

                    // read imu data
imu_data = imu.getImuData();

        // acceleration in meters per second squared in three axes
float acc_x = imu_data.acc(0);
float acc_y = imu_data.acc(1);
float acc_z = imu_data.acc(2);

//printf("acc_x: %f, acc_y: %f, acc_z: %f\n", acc_x, acc_y, acc_z);
printf(">acc_x:%f\n>acc_y:%f\n>acc_z:%f\n", acc_x, acc_y, acc_z);


// pitch, roll, yaw according to Tait-Bryan angles ZXY
// where R = Rz(yaw) * Rx(roll) * Ry(pitch)
// singularity at roll = +/-pi/2
float pitch = imu_data.pry(0);
float roll = imu_data.pry(1);
float yaw = imu_data.pry(2);

//printf("pitch: %f, roll: %f, yaw: %f\n", pitch, roll, yaw);
printf(">pitch:%f\n>roll:%f\n>yaw:%f\n", pitch, roll, yaw);




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

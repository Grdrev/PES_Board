#include "mbed.h"
#include <Eigen/Dense>
#include "DCMotor.h"
#include "SensorBar.h"
#include "Servo.h"
#include "IMU.h"
#include "ColorSensor.h"
#include <cstring>
#include "PESBoardPinMap.h"
#include "DebounceIn.h"
#include <cstdio>
// ----------------------------------------------------------------------------------------
bool do_execute_main_task = false; // toggled via the user button (blue button)
bool do_reset_all_once = false;    // runs a reset block exactly once per re-arm
//--------------------------------------------------------------------------------------------
DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();
//------------------------------------------------------------------------------
static const int   TOTAL_STOPS          = 8;    // equal to the total number of stops (4 colors x pickup+delivery)
static const float CENTER_DETECT_THRESH = 0.90f;
static const float LEFT_LEDS_OFF_THRESH = 0.15f;
static const int   NUM_COLORS           = 4;    // RED, YELLOW, GREEN, BLUE
#define M_PIf 3.14159265358979323846f

#include <cstdio>

// Get access to the existing console interface used by printf
static FileHandle *pc = mbed_file_handle(STDIN_FILENO);

static char rx_buffer[32];
static int rx_index = 0;

void handle_serial_commands();

enum RobotState {SLEEP, FOLLOW_LINE, AT_JUNCTION, LEAVE_JUNCTION, TURN_LEFT, UNLOADING, DONE};
enum class PackageAction {NONE, PICKUP, DELIVERY};
enum class BoxColor {RED = 0, YELLOW = 1, GREEN = 2, BLUE = 3, INVALID = -1}; // Compact color identity used to index boxes / visit counts (0..3). independent of the library assigned numbers 

static BoxColor colorNumToBoxColor(int color_num)
{
    switch (color_num) {
        case 3: return BoxColor::RED;
        case 4: return BoxColor::YELLOW;
        case 5: return BoxColor::GREEN;
        case 7: return BoxColor::BLUE;
        default: return BoxColor::INVALID;
    }
}

static bool isValidPackageColor(const char* colorStr)
{
    return strcmp(colorStr, "YELLOW") == 0 || strcmp(colorStr, "RED") == 0 ||
           strcmp(colorStr, "GREEN")  == 0 || strcmp(colorStr, "BLUE") == 0;
}

RobotState robot_state = RobotState::SLEEP;
static void stop_motors(DCMotor &motor_M1, DCMotor &motor_M2, DigitalOut &enable_motors);
//-------------------------------------------------------------------------------------------------------------

struct ArmPose {float base; float shoulder; float wrist;};

struct DeliveryContext {
    int number_of_stops = 0;
    bool turned_left_already = false;
    int color_visit_count[NUM_COLORS] = {0, 0, 0, 0};
    bool package_picked_up[NUM_COLORS] = {false, false, false, false};
    PackageAction current_action = PackageAction::NONE;
    int selected_box_index = -1; // which os the colours has been indetified during the stops 
    ArmPose selected_pickup_ground_pose = {0.0f, 0.0f, 0.0f};   // ground pose to grab from during pickup
    ArmPose selected_delivery_ground_pose = {0.0f, 0.0f, 0.0f}; // ground pose to place at during delivery
};
DeliveryContext context;

// -----------------------------------------------------------------------
static void arm_move_to_pose(Servo &base, Servo &shoulder, Servo &wrist, const ArmPose &pose);
static void arm_pickup(Servo &base, Servo &shoulder, Servo &wrist,
                        const ArmPose &ground_pose, const ArmPose &box_pose, const ArmPose &home_pose);
static void arm_deliver(Servo &base, Servo &shoulder, Servo &wrist,
                         const ArmPose &box_pose, const ArmPose &ground_pose, const ArmPose &home_pose);
static void driveAlongLine(DCMotor &motor_M1, DCMotor &motor_M2, SensorBar &sensorBar,
                            const Eigen::Matrix2f &Cwheel2robot_inv, float forward_gain,
                            float wheel_vel_max, float r_wheel, float Kp, float Kp_nl, float speed_factor);
                       
static void driveWithAngle(DCMotor &motor_M1, DCMotor &motor_M2, const Eigen::Matrix2f &Cwheel2robot_inv, float forward_gain,
                            float wheel_vel_max, float r_wheel, float Kp, float Kp_nl,
                            float speed_factor, float angle);


int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20;
    Timer main_task_timer;

    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f;
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;
    const float r_wheel = 0.0325f;
    const float b_wheel = 0.1735f;
    const float bar_dist = 0.055f;

    DCMotor motor_M2(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M1(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f);
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.3f);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.3f);

    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r_wheel / 2.0f, r_wheel / 2.0f, r_wheel / b_wheel, -r_wheel / b_wheel;
    const Eigen::Matrix2f Cwheel2robot_inv = Cwheel2robot.inverse();

    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    float speed_factor;
    int step_latch_counter = 0;
    SensorBar sensorBar(PB_9, PB_8, bar_dist);
    const float Kp = 2.5f * 2.0f;
    const float Kp_nl = 0.3f * 17.0f;

    // servo motor data
    Servo servo_ball(PB_D3);
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    float servo_ball_ang_min = 0.0325f;
    float servo_ball_ang_max = 0.1175f;
    servo_ball.calibratePulseMinMax(servo_ball_ang_min, servo_ball_ang_max);
    const float servo_ball_ang_range_min = -M_PIf / 2.0f;
    const float servo_ball_ang_range_max = M_PIf / 2.0f;
    const float normalised_angle_gain = 1.0f / M_PIf;
    const float normalised_angle_offset = 0.5f;
    static float pitch_servo_width = 0.5f;
    servo_ball.setPulseWidth(pitch_servo_width);
    const float Ts = static_cast<float>(main_task_period_ms) / 1000.0f;
    const float kp_ball = 7.0f;
    static float gyro_bias_estimate = 0.0f; // add near pitch_estimate declaration
    const float ki_ball = 0.5f; // tune this — start small, integral terms are easy to destabilize
    const float acc_trust_thresh_g = 0.3f * 9.81f; // Scaled to ~2.94 m/s^2 to match your IMU units; // how far ||acc|| can deviate from 1g before trust hits 0
    float pitch_estimate = 0.0f; 
    servo_ball.setMaxAcceleration(0.4f);
    // -------------------------------------------Colour sensor-------------------
    float color_cal[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int color_num = 0;
    const char* color_string = "";
    ColorSensor Color_Sensor(PB_3);

    // ---------------------------------arm calibrations --------------------------------------------------------------
    Servo servo_base(PB_D2);
    Servo servo_shoulder(PB_D1);
    Servo servo_wrist(PB_D0);
    servo_base.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_shoulder.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_wrist.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_base.setMaxAcceleration(0.05f);
    servo_shoulder.setMaxAcceleration(0.05f);
    servo_wrist.setMaxAcceleration(0.05f);

    // --------------------------------------Desired positions---------------------------------------
    ArmPose HOME_POSE = {0.00f, 0.00f, 0.8f};
    ArmPose PICKUP_POSE[NUM_COLORS] = {
        {0.00f, 0.00f, 0.8f},  // RED    
        {0.00f, 0.00f, 0.8f},  // YELLOW 
        {0.00f, 0.00f, 0.8f},  // GREEN  
        {0.00f, 0.00f, 0.8f},  // BLUE  
    };
    ArmPose DELIVERY_POSE[NUM_COLORS] = {
        {0.00f, 0.00f, 0.8f},  // RED    
        {0.00f, 0.00f, 0.8f},  // YELLOW 
        {0.00f, 0.00f, 0.8f},  // GREEN  
        {0.00f, 0.00f, 0.8f},  // BLUE 
    };

    // The robot has one box dedicated per package based on color 
    ArmPose BOX_POSE[NUM_COLORS] = {
        {0.75f, 0.55f, 0.2f},  // RED   
        {0.85f, 0.55f, 0.2f},  // YELLOW 
        {0.89f, 0.49f, 0.2f},  // GREEN 
        {1.0f, 0.3f, 0.25f},  // BLUE   
    };
// ----------------------------step detection and response 
// ----------------------------step detection and response 
// 0.25g converted to m/s^2 is roughly 2.45 m/s^2. 
        // Your serial log had a threshold of 0.25; let's scale it properly.
        const float step_acc_thresh_g   = 2.45f;   // Triggers if deviation exceeds ~2.45 m/s^2
        const float step_gyro_thresh    = 1.00f;   // Matches your serial log's working gyro threshold
        const float step_speed_factor   = 0.13f;   
        const float baseline_alpha      = 0.01f;   
        static float acc_norm_baseline  = 9.81f;  // Keeps baseline centered around 1g in m/s^2  // forced speed_factor while recovering from a step
        const int   STEP_LATCH_ITERS   = 25;     // ~500ms at 20ms loop -- how long the slowdown holds after detection
        float last_valid_angle = 0.0f;

        if (pc) {
        pc->set_blocking(false);
    }
// ------------------------------------------------------------------------------------------------------------
    main_task_timer.start();
    while (true) {
        main_task_timer.reset();
        if (!servo_ball.isEnabled()) servo_ball.enable();
        imu_data = imu.getImuData();

        if (do_execute_main_task) {
            //--------------------------------------------------------- Ball handling dynamics -------------------------------------------
            const float acc_norm  = imu_data.acc.norm();                 // magnitude of measured accel, ~1g when steady
            const float acc_dev    = fabsf(acc_norm - acc_norm_baseline);              // deviation from 1g -> signals bump/impact
            const float accel_trust = 1.0f - fminf(acc_dev / acc_trust_thresh_g, 1.0f); // 1 = fully trust accel, 0 = ignore it
            const float pitch_acc = atan2f(-imu_data.acc(1), imu_data.acc(2)); // pitch angle from accelerometer
            const float pitch_error = pitch_acc - pitch_estimate;
            gyro_bias_estimate += Ts * ki_ball * accel_trust * pitch_error; // only integrate when accel is trustworthy
            pitch_estimate += Ts * (imu_data.gyro(0) + gyro_bias_estimate + kp_ball * accel_trust * pitch_error);
            float clamped_pitch = pitch_estimate;
            if (clamped_pitch < servo_ball_ang_range_min) clamped_pitch = servo_ball_ang_range_min;
            if (clamped_pitch > servo_ball_ang_range_max) clamped_pitch = servo_ball_ang_range_max;
            pitch_servo_width = normalised_angle_gain * (-clamped_pitch) + normalised_angle_offset;
            servo_ball.setPulseWidth(pitch_servo_width);

            Color_Sensor.switchLed(ON);
            for (int i = 0; i < 4; i++) { color_cal[i] = Color_Sensor.readColorCalib()[i]; }
            color_num = Color_Sensor.getColor();
            color_string = Color_Sensor.getColorString(color_num);
            bool color_detected = isValidPackageColor(color_string);

            // --- step / bump detection (reuses acc_dev computed above) ---
        // --- step / bump detection (reuses acc_dev computed above) ---
        const float gyro_mag = fabsf(imu_data.gyro(0));

        // Ignore gyro-based triggering while deliberately turning -- a turn produces
        // a large angular rate on purpose, that is not a step/bump.
        bool gyro_flags_step = (gyro_mag > step_gyro_thresh) && (robot_state != RobotState::TURN_LEFT);
        bool step_detected = (acc_dev > step_acc_thresh_g) || gyro_flags_step;
        
        if (step_detected) {
            step_latch_counter = STEP_LATCH_ITERS;
        } else if (step_latch_counter > 0) {
            step_latch_counter--;
        }

        bool in_step_recovery = (step_latch_counter > 0);

        if (in_step_recovery) {
            speed_factor = step_speed_factor;          // slow down for the step
        } else if (context.turned_left_already) {
            speed_factor = 0.30f;                       // normal cruising speed after the turn
        } else {
            speed_factor = 0.15f;                       // slow speed before the first turn
        }
        if (!step_detected) {
                acc_norm_baseline += baseline_alpha * (acc_norm - acc_norm_baseline);
         }
        printf("acc_dev: %.3f (thresh %.2f) | gyro_mag: %.3f (thresh %.2f) | acc_trig: %d | gyro_trig: %d | latch: %d | sf: %.2f\n",
       acc_dev, step_acc_thresh_g, gyro_mag, step_gyro_thresh,(acc_dev > step_acc_thresh_g), gyro_flags_step, step_latch_counter, speed_factor);



            float centre = sensorBar.getMeanFourAvgBitsCenter();
            bool junction_detected = (centre >= CENTER_DETECT_THRESH);

            // state machine
            switch (robot_state) {
                case RobotState::SLEEP: {
                    stop_motors(motor_M1, motor_M2, enable_motors);
                    robot_state = RobotState::FOLLOW_LINE;
                    printf("starting to follow line\n");
                    break;
                }

                case RobotState::FOLLOW_LINE: {
                 enable_motors = 1;

                    if (!sensorBar.isAnyLedActive()) {
                                        if (in_step_recovery) {
                                            last_valid_angle *= 0.9f; // decay toward straight the longer we're blind
                                            driveWithAngle(motor_M1, motor_M2, Cwheel2robot_inv, 0.5f, wheel_vel_max,
                                                            r_wheel, Kp, Kp_nl, speed_factor, last_valid_angle);
                                        }
                                        break;
                                    }

                                    last_valid_angle = sensorBar.getAvgAngleRad();
                                    driveAlongLine(motor_M1, motor_M2, sensorBar, Cwheel2robot_inv,
                                                    0.5f, wheel_vel_max, r_wheel, Kp, Kp_nl, speed_factor);

                                    if (junction_detected) {
                                        if (!context.turned_left_already) {
                                            printf("First junction detected! Initiating Left Turn Loop...\n");
                                            context.turned_left_already = true;
                                            robot_state = RobotState::TURN_LEFT;
                                        } else {
                                            printf("Subsequent junction detected! Switching to AT_JUNCTION\n");
                                            robot_state = RobotState::AT_JUNCTION;
                                        }
                                    }
                                    break;
                                }

                case RobotState::TURN_LEFT: {
                    enable_motors = 1;
                    float angle = 0.0f;
                    if (sensorBar.isAnyLedActive()) {
                        angle = sensorBar.getAvgAngleRad();
                    }

                    Eigen::Vector2f robot_coord = {0.5f * wheel_vel_max * r_wheel,
                                                    Kp * angle + Kp_nl * angle * fabsf(angle)};
                    Eigen::Vector2f wheel_speed = Cwheel2robot_inv * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf) * speed_factor);
                    motor_M2.setVelocity(-wheel_speed(1) / (2.0f * M_PIf) * speed_factor * 0.2f);
                    if (!junction_detected && centre > LEFT_LEDS_OFF_THRESH) {
                        printf("Left turn complete. Back to line follow.\n");
                        robot_state = RobotState::FOLLOW_LINE;
                    }
                    break;
                }

                case RobotState::AT_JUNCTION: {
                    stop_motors(motor_M1, motor_M2, enable_motors);
                    thread_sleep_for(150); 
                    color_num = Color_Sensor.getColor();
                    color_string = Color_Sensor.getColorString(color_num);
                    printf("[AT JUNCTION] Stopped. Color Read: %d (%s)\n", color_num, color_string);

                    BoxColor box_color = colorNumToBoxColor(color_num);
                    bool valid_color = (box_color != BoxColor::INVALID) && isValidPackageColor(color_string);

                    if (valid_color) {
                        context.selected_box_index = static_cast<int>(box_color);
                        int idx = context.selected_box_index;
                        context.selected_pickup_ground_pose = PICKUP_POSE[idx];
                        context.selected_delivery_ground_pose = DELIVERY_POSE[idx];

                        if (context.color_visit_count[idx] == 0) {
                            context.current_action = PackageAction::PICKUP;
                            printf("[AT JUNCTION] Color %s seen for the 1st time -> PICKUP into box %d\n", color_string, idx);
                        } else {
                            context.current_action = PackageAction::DELIVERY;
                            printf("[AT JUNCTION] Color %s seen for the 2nd time -> DELIVERY from box %d\n", color_string, idx);
                        }
                        context.color_visit_count[idx]++;

                        robot_state = (context.number_of_stops < TOTAL_STOPS) ? RobotState::UNLOADING : RobotState::DONE;
                    } else {
                        printf("[WARNING] Stopped but color invalid. Resuming journey...\n");
                        robot_state = RobotState::LEAVE_JUNCTION;
                    }
                    break;
                }

                case RobotState::UNLOADING: {
                    stop_motors(motor_M1, motor_M2, enable_motors);

                    if (!servo_base.isEnabled()) servo_base.enable();
                    if (!servo_shoulder.isEnabled()) servo_shoulder.enable();
                    if (!servo_wrist.isEnabled()) servo_wrist.enable();

                    int idx = context.selected_box_index;
                    if (idx >= 0 && idx < NUM_COLORS) {
                        if (context.current_action == PackageAction::PICKUP) {
                            printf("PICKUP: storing %s package into box %d.\n", color_string, idx);
                            arm_pickup(servo_base, servo_shoulder, servo_wrist,context.selected_pickup_ground_pose, BOX_POSE[idx], HOME_POSE);
                            context.package_picked_up[idx] = true;
                        } else if (context.current_action == PackageAction::DELIVERY) {
                            printf("DELIVERY: retrieving %s package from box %d.\n", color_string, idx);
                            arm_deliver(servo_base, servo_shoulder, servo_wrist, BOX_POSE[idx], context.selected_delivery_ground_pose, HOME_POSE);
                            context.package_picked_up[idx] = false;
                        }
                    }

                    thread_sleep_for(1000);
                    context.number_of_stops++;
                    context.current_action = PackageAction::NONE;
                    robot_state = RobotState::LEAVE_JUNCTION;
                    break;
                }

                case RobotState::LEAVE_JUNCTION: {
                    enable_motors = 1;
                    if (!sensorBar.isAnyLedActive()) break;
                    driveAlongLine(motor_M1, motor_M2, sensorBar, Cwheel2robot_inv, 0.4f, wheel_vel_max, r_wheel, Kp, Kp_nl, speed_factor);
                    
                    if (!junction_detected && !color_detected) {
                        robot_state = RobotState::FOLLOW_LINE;
                    }
                    break;
                }

                case RobotState::DONE: {
                    printf("DONE\n");
                    stop_motors(motor_M1, motor_M2, enable_motors);
                    break;
                }

                default:
                    break;
            }

        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;

                stop_motors(motor_M1, motor_M2, enable_motors);
                context = DeliveryContext{};
                robot_state = RobotState::SLEEP;
                printf("RESETTING VARIABLES AND OBJECTS\n");
                enable_motors = 0;
                pitch_servo_width = 0.0f;
                servo_ball.setPulseWidth(pitch_servo_width);
                servo_base.setPulseWidth(HOME_POSE.base);
                servo_shoulder.setPulseWidth(HOME_POSE.shoulder);
                servo_wrist.setPulseWidth(HOME_POSE.wrist);
            }
        }

        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
        {
            do_execute_main_task = !do_execute_main_task;
            if (do_execute_main_task)
                do_reset_all_once = true;
        }

static void stop_motors(DCMotor &motor_M1, DCMotor &motor_M2, DigitalOut &enable_motors)
        {
            motor_M1.setVelocity(0.0f);
            motor_M2.setVelocity(0.0f);
            enable_motors = 0;
        }

static void driveAlongLine(DCMotor &motor_M1, DCMotor &motor_M2, SensorBar &sensorBar,
                            const Eigen::Matrix2f &Cwheel2robot_inv, float forward_gain,
                            float wheel_vel_max, float r_wheel, float Kp, float Kp_nl, float speed_factor)
        {
            float angle = sensorBar.getAvgAngleRad();
            Eigen::Vector2f robot_coord = {forward_gain * wheel_vel_max * r_wheel,
                                            Kp * angle + Kp_nl * angle * fabsf(angle)};
            Eigen::Vector2f wheel_speed = Cwheel2robot_inv * robot_coord;
            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf) * speed_factor);
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf) * speed_factor);
        }

static void arm_move_to_pose(Servo &base, Servo &shoulder, Servo &wrist, const ArmPose &pose)
        {
            base.setPulseWidth(pose.base);
            thread_sleep_for(500);
            shoulder.setPulseWidth(pose.shoulder);
            thread_sleep_for(500);
            wrist.setPulseWidth(pose.wrist);
            thread_sleep_for(500);
        }

static void arm_pickup(Servo &base, Servo &shoulder, Servo &wrist, const ArmPose &ground_pose, const ArmPose &box_pose, const ArmPose &home_pose)
        {
            arm_move_to_pose(base, shoulder, wrist, ground_pose); // reach down and grab the package
            thread_sleep_for(500);                                 // let the magnet/gripper attach
            arm_move_to_pose(base, shoulder, wrist, box_pose);     // move it into its on-robot box
            thread_sleep_for(500);                                 // let it release into the box
            arm_move_to_pose(base, shoulder, wrist, home_pose);
        }

static void arm_deliver(Servo &base, Servo &shoulder, Servo &wrist,const ArmPose &box_pose, const ArmPose &ground_pose, const ArmPose &home_pose)
        {
            arm_move_to_pose(base, shoulder, wrist, box_pose);     // reach into the box and grab the package
            thread_sleep_for(500);
            arm_move_to_pose(base, shoulder, wrist, ground_pose);  // place it down for delivery
            thread_sleep_for(500);
            arm_move_to_pose(base, shoulder, wrist, home_pose);
        }

static void driveWithAngle(DCMotor &motor_M1, DCMotor &motor_M2, const Eigen::Matrix2f &Cwheel2robot_inv, float forward_gain,
                            float wheel_vel_max, float r_wheel, float Kp, float Kp_nl,
                            float speed_factor, float angle)
        {
            Eigen::Vector2f robot_coord = {forward_gain * wheel_vel_max * r_wheel,
                                            Kp * angle + Kp_nl * angle * fabsf(angle)};
            Eigen::Vector2f wheel_speed = Cwheel2robot_inv * robot_coord;
            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf) * speed_factor);
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf) * speed_factor);
        }


void handle_serial_commands() 
{
    if (!pc) return;

    char c;
    // read() returns the number of bytes read, or a negative error code if empty
    while (pc->read(&c, 1) > 0) {
        
        // If it's a newline or carriage return, process the command
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Null-terminate string

                if (strcmp(rx_buffer, "START") == 0) {
                    if (!do_execute_main_task) {
                        do_execute_main_task = true;
                        do_reset_all_once = true;
                        printf("[SERIAL] Starting execution\n");
                    }
                } 
                else if (strcmp(rx_buffer, "STOP") == 0) {
                    do_execute_main_task = false;
                    do_reset_all_once = true;
                    printf("[SERIAL] Stopping execution\n");
                } 
                else if (strcmp(rx_buffer, "RESET") == 0) {
                    do_execute_main_task = false;
                    do_reset_all_once = true;
                    printf("[SERIAL] Resetting system\n");
                } 
                else {
                    printf("[SERIAL] Unknown command: %s\n", rx_buffer);
                }

                rx_index = 0; // Reset for next command
            }
        } 
        // Build up the string until buffer is full
        else if (rx_index < (int)(sizeof(rx_buffer) - 1)) {
            rx_buffer[rx_index++] = c;
        }
    }
}
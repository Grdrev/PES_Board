#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "Servo.h"     // Custom Servo library header
#include <cstdio>
#include <cstring>

// --- Interrupt & Serial Configuration ---
// Define raw, unbuffered serial on ST-LINK pins for strict hardware ISR management
UnbufferedSerial pc(USBTX, USBRX, 115200);

// Use EventQueue to safely offload full string parsing out of the hardware ISR context
EventQueue queue(32 * EVENTS_EVENT_SIZE);

// Shared volatile variables across ISR and main thread barriers
char rx_buffer[64];
volatile int rx_index = 0;

volatile bool do_execute_main_task = false; 
volatile bool do_reset_all_once = false;    

// Global array tracking target servo states
volatile float pw[] = {0.00f, 0.00f, 0.0f}; 
int loop_counter = 0;

// Objects for user button handling
DebounceIn user_button(BUTTON1);   
void toggle_do_execute_main_fcn(); 

// Function Prototypes
void rx_interrupt();
void process_serial_command();

int main()
{
    // Attach hardware interrupt vector handlers
    user_button.fall(&toggle_do_execute_main_fcn);
    pc.attach(&rx_interrupt, UnbufferedSerial::RxIrq);

    // Timing parameters for the main periodic physical loop
    const int main_task_period_ms = 20; 
    Timer main_task_timer;              

    DigitalOut user_led(LED1);

    // Initialize 3-axis robot arm components mapping physical board pins
    Servo servo_base(PB_D2);
    Servo servo_shoulder(PB_D1);
    Servo servo_wrist(PB_D0);

    struct ArmPose { float base; float shoulder; float wrist; };
    const ArmPose HOME_POSE = {0.00f, 0.00f, 0.8f};

    // Initialize and calibrate servo boundaries
    servo_base.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_shoulder.calibratePulseMinMax(0.0325f, 0.1175f);
    servo_wrist.calibratePulseMinMax(0.0325f, 0.1175f);
    
    servo_base.setMaxAcceleration(0.15f);
    servo_shoulder.setMaxAcceleration(0.15f);
    servo_wrist.setMaxAcceleration(0.15f);

    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        // ARCHITECTURE CRITICAL: Execute any deferred serial commands waiting in the queue.
        // dispatch_for(0) forces an instant poll and flush of outstanding events without blocking.
        queue.dispatch_for(0ms);

        // Throttle feedback telemetries to print once every 2.5 seconds (12 * 200ms)
        if (loop_counter >= 12) {
            printf("[pw] B: %.3f, Sh: %.3f, Wr: %.3f\r\n", pw[0], pw[1], pw[2]);
            loop_counter = 0; 
        }
        loop_counter++;

        if (do_execute_main_task) {
            if (!servo_base.isEnabled()) servo_base.enable();
            if (!servo_shoulder.isEnabled()) servo_shoulder.enable();
            if (!servo_wrist.isEnabled()) servo_wrist.enable();

            // Continuously stream the latest volatile pulse-widths to the hardware timers
            servo_base.setPulseWidth(pw[0]);
            servo_shoulder.setPulseWidth(pw[1]);
            servo_wrist.setPulseWidth(pw[2]);
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;

                pw[0] = HOME_POSE.base;
                pw[1] = HOME_POSE.shoulder;
                pw[2] = HOME_POSE.wrist;

                servo_base.setPulseWidth(HOME_POSE.base);
                servo_shoulder.setPulseWidth(HOME_POSE.shoulder);
                servo_wrist.setPulseWidth(HOME_POSE.wrist);
            }
        }

        user_led = !user_led;

        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0) {
            printf("Warning: Main task took longer than main_task_period_ms\n");
        } else {
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
        }
    }
}

// --- Interrupt Handling Mechanics ---

void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task) {
        do_reset_all_once = true;
    }
}

// 1. HARDWARE ISR CONTEXT: Fast capture. Never do prints or string logic inside here.
void rx_interrupt() 
{
    char c;
    if (pc.read(&c, 1)) {
        // Echo back incoming raw keystrokes immediately
        pc.write(&c, 1);

        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; 
                // Defer parsing away from the strict hardware interrupt context onto the thread queue
                queue.call(process_serial_command);
            }
        } 
        else if (rx_index < (int)(sizeof(rx_buffer) - 1)) {
            if (c != '\b' && c != '\t') {
                rx_buffer[rx_index++] = c;
            }
        }
    }
}

// 2. MAIN THREAD CONTEXT: Triggered safely via EventQueue dispatching
void process_serial_command()
{
    // --- String Command Routing Logic ---
    if (strcmp(rx_buffer, "START") == 0) {
        if (!do_execute_main_task) {
            do_execute_main_task = true;
            do_reset_all_once = true;
            printf("\r\n[PES] Interrupt Start Confirmed...\r\n");
        }
    } 
    else if (strcmp(rx_buffer, "STOP") == 0) {
        do_execute_main_task = false;
        do_reset_all_once = true;
        printf("\r\n[PES] Interrupt Stop Confirmed...\r\n");
    } 
    else if (strncmp(rx_buffer, "servo:", 6) == 0) {
        float val1 = 0.0f, val2 = 0.0f, val3 = 0.0f;
        int parsed = sscanf(rx_buffer, "servo: %f%*[,]%f%*[,]%f", &val1, &val2, &val3);
        
        if (parsed == 3) {
            printf("\r\n[PES] Parsed Interrupt Servos -> Base: %.3f, Sh: %.3f, Wr: %.3f\r\n", val1, val2, val3);
            pw[0] = val1; 
            pw[1] = val2; 
            pw[2] = val3;
        } else {
            printf("\r\n[PES] Error: Invalid servo parsing structure.\r\n");
        }
    }
    else {
        printf("\r\n[PES] Unknown Interrupt Command Sequence: %s\r\n", rx_buffer);
    }

    // Flush and reset the index pointer to prepare for the next message packet
    rx_index = 0; 
}
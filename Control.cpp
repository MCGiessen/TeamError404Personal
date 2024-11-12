
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include "Global.h"
#include "Control.h"

// GPIO pin assignments and frequencies
const unsigned int Throttle_Pin = 12;
const unsigned int Steer_Pin = 13;
const unsigned int Throttle_Freq = 50;     // Throttle control frequency in Hz
const unsigned int Steer_Freq = 240;       // Steering control frequency in Hz


// Maps the control input range [-1, 1] to the duty cycle range [800, 2400] microseconds
unsigned int map_input_to_us(double input) {
    return 1600 + (input * 800);  // 1600 us is neutral; 
}

// Function to apply a single high-low PWM pulse with a calculated duty cycle
void apply_pwm_pulse(struct gpiod_line* line, unsigned int high_time_us, unsigned int frequency) {
    unsigned int period_us = 1000000 / frequency;

    // Apply a single high-low PWM cycle
    if (gpiod_line_set_value(line, 1) < 0) {
        perror("Failed to set line high");
    }
    usleep(high_time_us);  // High time

    if (gpiod_line_set_value(line, 0) < 0) {
        perror("Failed to set line low");
    }
    usleep(period_us - high_time_us);  // Low time
}

// Throttle control thread function
void* throttle_control(void* arg) {
    //printf("throttle called");
    struct gpiod_chip* chip = (struct gpiod_chip*)arg;

    // Open throttle GPIO line once at the start
    struct gpiod_line* throttle_line = gpiod_chip_get_line(chip, Throttle_Pin);
    if (!throttle_line || gpiod_line_request_output(throttle_line, "pwm_control", 0) < 0) {
        perror("Failed to set up throttle line");
        return NULL;
    }

    // Main loop to apply PWM signals for throttle
    while (running) {
        //printf("throttle working\n");
        unsigned int duty_cycle_us = map_input_to_us(throttle_input);
        apply_pwm_pulse(throttle_line, duty_cycle_us, Throttle_Freq);
    }

    // Release the GPIO line when exiting
    gpiod_line_release(throttle_line);
    return NULL;
}

// Steering control thread function
void* steering_control(void* arg) {
    printf("steering called\n");
    struct gpiod_chip* chip = (struct gpiod_chip*)arg;

    // Open steering GPIO line once at the start
    struct gpiod_line* steering_line = gpiod_chip_get_line(chip, Steer_Pin);
    if (!steering_line || gpiod_line_request_output(steering_line, "pwm_control", 0) < 0) {
        perror("Failed to set up steering line");
        return NULL;
    }

    float steering_input_adj = 0.35;

    // Main loop to apply PWM signals for steering
    while (running) {
        printf("steering working\n");
        unsigned int duty_cycle_us = map_input_to_us(steering_input_adj * steering_input);
        apply_pwm_pulse(steering_line, duty_cycle_us, Steer_Freq);
    }

    // Release the GPIO line when exiting
    gpiod_line_release(steering_line);
    return NULL;
}

// Main control function to set up the GPIO chip and launch control threads
void control() {

    printf("control called\n");
    // Open the GPIO chip
    struct gpiod_chip* chip = gpiod_chip_open_by_number(4);
    if (!chip) {
        perror("Failed to open gpiochip");
        return;
    }


    // Create threads for throttle and steering control
    pthread_t throttle_thread, steering_thread;
    pthread_create(&throttle_thread, NULL, throttle_control, chip);
    pthread_create(&steering_thread, NULL, steering_control, chip);

    
    // Wait for threads to finish (for example, when SIGINT is received)
    pthread_join(throttle_thread, NULL);
    pthread_join(steering_thread, NULL);

    // Close the GPIO chip once both threads have finished
    gpiod_chip_close(chip);
    printf("Control system exited.\n");
}

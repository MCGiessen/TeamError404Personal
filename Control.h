#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <pthread.h>

// GPIO pin assignments and frequencies
extern const unsigned int Throttle_Pin;
extern const unsigned int Steer_Pin;
extern const unsigned int Throttle_Freq;
extern const unsigned int Steer_Freq;


// Signal handler to exit loop gracefully on SIGINT (e.g., CTRL+C)
void handle_sigint(int sig);

// Maps the control input range [-1, 1] to the duty cycle range [800, 2400] microseconds
unsigned int map_input_to_us(double input);

// Function to apply a single high-low PWM pulse with a calculated duty cycle
void apply_pwm_pulse(struct gpiod_line* line, unsigned int high_time_us, unsigned int frequency);

// Throttle control thread function
void* throttle_control(void* arg);

// Steering control thread function
void* steering_control(void* arg);

// Main control function to set up the GPIO chip and launch control threads
void control();

#endif // CONTROL_H

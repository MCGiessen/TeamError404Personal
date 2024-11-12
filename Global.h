#pragma once

#include <iostream>
#include <vector>
#include <signal.h>

extern double steering_input;
extern double throttle_input;

extern bool running;
extern bool nav_flag;
extern bool data_flag;
extern bool control_flag;

//extern int data_index;

struct data_point
{
	float x;
	float y;
	float h;
	float t; // Timestamp
};
extern data_point current_data;

struct Point {
	double x, y;
};

extern std::vector<Point> Course;

//extern std::vector<data_point> data;
//extern int data_index;

constexpr float N_PI = 3.1415f;
constexpr float TWO_PI = 6.2831f;
constexpr int MAX_PATH_LENGTH = 200000;

// Signal handler to exit loop gracefully on SIGINT (e.g., CTRL+C)
void handle_sigint(int sig);

void get_data();

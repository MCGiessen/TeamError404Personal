
#pragma once

#include <chrono>
#include <iostream>
#include "Global.h"

//#include <cmath>
//#include <iostream>
//#include "globals.h"
//#include "car.h"
// Structure to store positional data and timestamp

class Car;

struct location {
    double x, y, h, t;
};

class CarData {
public:
    bool navRead;
    //location loc[200000];
    int i, j, k;
    //location velocity;
    location* displacement;
    double* mag_displace;
    int turn_flag = 0;
    int loop_count = 0;
    //int wFlag = 0;
    std::chrono::steady_clock::time_point start_time;


    // Constructor
    CarData(double x, double y, double h);

    ~CarData();

    // Method to update car data (position and velocity)
    void update(Point currentWaypoint);

    // Retrieve the current position
    location get_value();

    // Calculate the distance between the car and a given waypoint
    double get_distance(Point waypoint);

    // Retrieve the current velocity
    location get_velocity();

    // Calculate the orientation relative to a waypoint
    int get_orientation(Point current_p);

    // Calculate the deviation from the waypoint
    double get_deviation(Point current_p);

    // Decide how much to turn based on distance and deviation
    void decide_turn(Point current_p);

    // Track the Instantaneous Center of Rotation (ICR)
    location track_ICR();

    double get_distance_between(location center, Point current_p);

    void Main_Update(Point current_p);

    void handle_waypoints();

    void change_heading();


private:
    // Circular buffer to store the last 200000 locations
    location* loc;

    // Indices for tracking positions in the circular buffer
    //int i, j, k;

    // Store velocity as x, y components and time
    location velocity;
};


void navigation_loop();




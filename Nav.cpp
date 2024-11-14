#pragma once

//
//  main.cpp
//  Senior_design code
//
//  Created by Luke Ozment on 9/29/24.
//

#include <chrono>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include "Nav.h"
#include "Global.h"


//Nav Variables
//double total_time = 0;

int wFlag = 0;

bool skip = 0;

Point currentWaypoint; //= Course[wFlag];

//Constructor
CarData::CarData(double x, double y, double h) 
    : start_time(std::chrono::steady_clock::now()) {
    displacement = new location[200000];
    mag_displace = new double[200000];
    loc = new location[200000];
    i = 0;
    j = 0;
    k = 0;
    loc[i].x = x;
    loc[i].y = y;
    loc[i].h = h;

    velocity.x = 0.0f;
    velocity.y = 0.0f;
    velocity.t = 0.0f;

    mag_displace[i] = get_distance_between(loc[i], currentWaypoint);
    displacement[i].x = currentWaypoint.x - loc[i].x;
    displacement[i].y = currentWaypoint.y - loc[i].y;
    displacement[i].t = 0;
    //printf("displacement: %f\n", mag_displace[i]);

}

//Destructor
CarData::~CarData()
{
    delete[] loc;
    delete[] displacement;
    delete[] mag_displace;
}

int c = 0;

//update object data
void CarData::update(Point currentWaypoint) {
        // Calculate the elapsed time as a double in seconds
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;

        // Record the elapsed time as a double
        double elapsed_seconds = elapsed.count();

        i = (i + 1) % 200000;
        loc[i].x = current_data.x;
        loc[i].y = current_data.y;
        //loc[i].h = current_data.h;
        loc[i].h = 2*acos(current_data.h);
        loc[i].t = elapsed_seconds;
    
    c++;
    if(c%100==0)
      printf("heading: %f\n", loc[i].h);
    
    
        //calculate new velocity
        j = (i - 1) % 200000;
        velocity.x = (loc[i].x - loc[j].x) / (loc[i].t - loc[j].t);
        velocity.y = (loc[i].y - loc[j].y) / (loc[i].t - loc[j].t);
        velocity.t = loc[i].t - loc[j].t;

        //calculate new displacement
        displacement[i].x = currentWaypoint.x - loc[i].x;
        displacement[i].y = currentWaypoint.y - loc[i].y;
        displacement[i].t = elapsed_seconds;
        mag_displace[i] = get_distance_between(loc[i], currentWaypoint);
        mag_displace[i] = get_distance(currentWaypoint);

        //printf("displacement: %f\n", mag_displace[i]);
    
        return;
    
}

location CarData::get_value() {

    location position;
    position.x = loc[i].x;
    position.y = loc[i].y;
    position.h = loc[i].h;
    position.t = loc[i].t;
    return position;
}

//get distance between car and waypoint
double CarData::get_distance(Point waypoint) {
    ;
    double x = waypoint.x - loc[i].x;
    double y = waypoint.y - loc[i].y;

    return sqrt(x * x + y * y);
}

//returns x velocity, y velocity, and delta t
location CarData::get_velocity() {
    return velocity;
}




//Deviation from the waypoint (at x and y location)
double CarData::get_deviation(Point current_p) {
    double a = loc[i].y - loc[j].y;
    double b = loc[i].x - loc[j].x;

    // Check if both a and b are zero (i.e., the points are the same)
    if (a == 0 && b == 0) {
        return 0.0;  // No deviation, return 0
    }

    double c = (loc[i].x * loc[j].y) - (loc[i].y * loc[j].x);
    double deviation = (abs((current_p.x * a) - (current_p.y * b) + c)) / sqrt((a * a) + (b * b));
    return deviation;
}


void CarData::decide_turn(Point current_p)
{
    j = (i == 0) ? 199999 : (i - 1);
    double deviation = get_deviation(current_p);
    //printf("Deviation: %f\n", deviation);

    double relative_heading = abs(asin(deviation / mag_displace[i]));

    //printf("Relative Heading: %f\n", relative_heading);

    if ((relative_heading <= 0.1) && ((mag_displace[i] - mag_displace[j]) < 0))
    {
     steering_input = 0;
     throttle_input = 1;
     return;
    }

    if ((relative_heading <= 0.3) && (relative_heading > 0.1))
    {
     steering_input = get_orientation(current_p) * 0.2f;
     return;
    }

    if (relative_heading > 0.3f)
    {
        //if (turn_flag == 3) { steeringInput = steeringInput / 2; return; }
        steering_input = get_orientation(current_p) * relative_heading;
        return;
    }

    if ((mag_displace[i] - mag_displace[j]) > 0)
    {
        //flag change heading
        turn_flag = 3;
       // printf("Turn Flag: %d \n", turn_flag);
        throttle_input = .5;
        steering_input = get_orientation(current_p);
    }
    else return;
}

//The last 3 locations form an arc. This returns the center of a circle that contains the 3 points.
location CarData::track_ICR() {
    j = (i == 0) ? 199999 : (i - 1);
    k = (i <= 1) ? 199999 + (i - 2) : (i - 2);

    location m1, m2;
    m1.x = (loc[k].x + loc[j].x) / 2;
    m1.y = (loc[k].y + loc[j].y) / 2;
    m2.x = (loc[j].x + loc[i].x) / 2;
    m2.y = (loc[j].y + loc[i].y) / 2;

    double mb1 = (loc[k].x - loc[j].x) / (loc[j].y - loc[k].y);
    double mb2 = (loc[j].x - loc[i].x) / (loc[i].y - loc[j].y);

    location c;
    c.x = ((mb2 * m2.x - mb1 * m1.x) - (m2.y - m1.y)) / (mb2 - mb1);
    c.y = (mb1 * (c.x - m1.x)) + m1.y;

    //c.t will be assigned the icr radius
    double x = c.x - m1.x;
    double y = c.y - m1.y;
    c.t = sqrt(x * x + y * y);
    return c;
}
//distance between icr and waypoint
double CarData::get_distance_between(location center, Point current_p)
{
    double x = center.x - current_p.x;
    double y = center.y - current_p.y;
    return sqrt(x * x + y * y);
}

//void change_heading()

void CarData::Main_Update(Point currentWaypoint)
{
    //update car object
    update(currentWaypoint);

    //if close to wall, reset
 //   if (loc[i].x < -6 || loc[i].x > 6 || loc[i].y < -10 || loc[i].y > 8)
 //   {
 //       turn_flag = 0;
 //   }
    
    update(currentWaypoint);
       decide_turn(currentWaypoint);
       return;
    
    /*
    if (turn_flag < 2) //going straight or small adjustment
    {
        steering_input = 0;
        decide_turn(currentWaypoint);
        return;
    }
    if (turn_flag == 2) //icr turn
    {
        loop_count = (loop_count + 1) % 300; //collect points along turn
        if (loop_count > 3)
        {
            location center = track_ICR();
            if (get_distance_between(center, currentWaypoint) > (center.t + .01)) //distance check
            {
                //adjust input
                steering_input = get_orientation(currentWaypoint) * .02 + steering_input;
                return;
            }
            if (get_distance_between(center, currentWaypoint) < (center.t - .01)) //distance check
            {
                //adjust input
                steering_input = steering_input - get_orientation(currentWaypoint) * .02;
                return;
            }
            else
                decide_turn(currentWaypoint);
            return;

        }
        else
        {
            return;
        }
    }
    if (turn_flag == 3)
    {
        //change heading

        decide_turn(currentWaypoint);
        return;
    }
    if (turn_flag > 3) { turn_flag = 0; return; }
     
     */
}

void CarData::handle_waypoints()
{
    //printf("displacement: %f", mag_displace[i]);
    if (mag_displace[i] < 0.1f)
    {
        printf("Displacement: %f\n", mag_displace[i]);
        printf("handled\n");
        wFlag = (wFlag + 1)%Course.size();
        printf("wFlag: %d\n", wFlag);
        currentWaypoint = Course[wFlag];
        turn_flag = 0;
        return;
    }
    else
    {
        return;
    }
}

int CarData::get_orientation(Point current_p)
{
    // Get the angle from the car to the current waypoint
    float dx = current_p.x - loc[i].x;
    float dy = current_p.y - loc[i].y;
    float waypoint_angle = atan2(dy, dx);  // Angle to the waypoint

    // Use the heading from OptiTrack data (in radians)
    //float car_heading_angle = myCar.getCurrentState().heading;  // Assuming optiTrack_heading stores the car's heading in radians

    float car_heading_angle = loc[i].h;



    // Calculate the difference between the waypoint angle and the car's heading
    float angle_diff = waypoint_angle - car_heading_angle;

    // Normalize angle_diff to be between -PI and PI
    while (angle_diff > N_PI) angle_diff -= 2 * N_PI;
    while (angle_diff < -N_PI) angle_diff += 2 * N_PI;

    // Decide whether to turn left or right based on the angle difference
    if (angle_diff > 0) return 1;  // Turn right
    else return -1;  // Turn left
}


//location normalize_vector(location vector)

void CarData::change_heading()
{

}



void navigation_loop()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(5));

    //using clock = std::chrono::steady_clock;  // Define the clock type
    
    // start time
    //auto programStart = clock::now();

    //create cardata object
    CarData this_car(current_data.x, current_data.y, current_data.h);

    currentWaypoint = Course[wFlag];

    //initial conditions
    throttle_input = 0;
    steering_input = 0;
    this_car.turn_flag = 0;

    //bool navRead = true;
    while (running)
    {
        //Will be updated later
        if (nav_flag == true)
        {

            throttle_input = .5;
            // Get the current time at the start of the loop
            //auto currentTime = clock::now();

            // Calculate the elapsed time since the program started
            //std::chrono::duration<double> elapsed = currentTime - programStart;
            //total_time = elapsed.count();
            //control_flag = false;
            //drive car
            this_car.Main_Update(currentWaypoint);
            //navRead = !navRead;
            //control_flag = true;
            //handle Waypoints
            this_car.handle_waypoints();
            usleep(4000);
        }


    }


}

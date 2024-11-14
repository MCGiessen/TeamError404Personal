#include "Global.h"
#include <signal.h>
#include <vector>

bool running = true;
bool nav_flag = true;
bool data_flag = true;
bool control_flag = true;

data_point current_data = { 0 };

std::vector<Point> Course = { {3,3},{-3,3},{-3,-3},{3,-3} };

//Final Course
/*
 std::vector<Point> Course = {
 {0.205f, -6.516f},
 {-3.558f, -5.775f},
 {-3.526f, -1.991f},
 {-1.095f, -0.26f},
 {-3.424f, 2.832f},
 {-2.27f, 5.314f},
 {1.131f, 5.394f},
 {4.112f, 4.59f},
 {4.25f, 1.452f},
 {2.238f, -1.52f},
 {4.224f, -4.417f}
 };
*/

//Course calculated in Matlab (doesn't work perfectly)
/*
 std::vector<Point> Course = {
{-3.6103f, -2.3082f},
{-1.5f, -0.81594f},
{-3.2209f, 2.3931f},
{-2.5403f, 5.0005f},
{1.0508f, 5.3993f},
{3.6827f, 4.8545f},
{4.2783f, 1.6565f},
{2.3924f, -1.062f},
{4.0408f, -3.7198f},
{0.40703f, -6.457f},
{-3.0906f, -6.0424f}
 };
*/

double steering_input = 0;
double throttle_input = 0;

// Signal handler to exit loop gracefully on SIGINT (e.g., CTRL+C)
void handle_sigint(int sig) {
    
    running = false;
}

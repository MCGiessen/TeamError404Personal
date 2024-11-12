#include "Global.h"
#include <signal.h>
#include <vector>

bool running = true;
bool nav_flag = true;
bool data_flag = true;
bool control_flag = true;

data_point current_data = { 0 };

std::vector<Point> Course = { {3,3},{-3,3},{-3,-3},{3,-3} };

double steering_input = 0;
double throttle_input = 0;

// Signal handler to exit loop gracefully on SIGINT (e.g., CTRL+C)
void handle_sigint(int sig) {
    
    running = false;
}

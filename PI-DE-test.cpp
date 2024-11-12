// PI-DE-test.cpp : Defines the entry point for the application.
//
#include <pthread.h>
#include <signal.h>
#include "PI-DE-test.h"
#include "Control.h"
#include "Global.h"
#include "Nav.h"
using namespace std;

int main()
{
    // Set up signal handling, if desired
    signal(SIGINT, handle_sigint);

    running = true;

    //optitrack
    pthread_t data;
    if (pthread_create(&data, NULL, (void* (*)(void*))get_data, NULL) != 0) {
        perror("Failed to create data thread");
        return 1;
    }


    //throttle_input = 1;
    //steering_input = 0;
    // Define a thread variable for the control loop
    pthread_t control_thread;

    // Assume handle_sigint is defined


    // Start the control loop in a new thread
    if (pthread_create(&control_thread, NULL, (void* (*)(void*))control, NULL) != 0) {
        perror("Failed to create control thread");
        return 1;
    }

    usleep(5000);
    pthread_t nav;

    if (pthread_create(&nav, NULL, (void* (*)(void*))navigation_loop, NULL) != 0) {
        perror("Failed to create control thread");
        return 1;
    }

    // Wait for the control thread to finish (optional)
    if (pthread_join(nav, NULL) != 0)
    {
        perror("Failed to join nav thread");
        return 1;
    }

    if (pthread_join(control_thread, NULL) != 0) {
        perror("Failed to join control thread");
        return 1;
    }
    if (pthread_join(data, NULL) != 0) {
        perror("Failed to join data thread");
        return 1;
        }
        printf("Control loop finished, exiting program.\n");
        return 0;

    
}


#include <iostream>
#include "Global.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>
#include <chrono>


//bool running = true;
// Define shared memory and semaphore names
const char* SHM_NAME = "data_arr";
const char* SEM_NAME = "/optitrack_semaphore";

// Define a struct to match the data layout in shared memory
struct OptiTrackData {
    float x;
    float y;
    float z;
    float heading;
};



void get_data() {

    // Record the start time of the program
    auto start_time = std::chrono::steady_clock::now();

    // Open the shared memory
    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory" << std::endl;
        return;
    }

    // Map the shared memory
    void* shm_addr = mmap(0, sizeof(OptiTrackData), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_addr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        close(shm_fd);
        return;
    }

    // Open the semaphore
    sem_t* sem = sem_open(SEM_NAME, 0);
    if (sem == SEM_FAILED) {
        std::cerr << "Failed to open semaphore" << std::endl;
        munmap(shm_addr, sizeof(OptiTrackData));
        close(shm_fd);
        return;
    }

    // Loop to read data from shared memory
    while (running) {
        bool nav_flag = false;
        // Wait for the semaphore to be released by the Python script
        sem_wait(sem);

        // Read data from shared memory
        OptiTrackData* data = static_cast<OptiTrackData*>(shm_addr);

        // Print the data
       // std::cout << "Position: (" << data->x << ", " << data->y << ", " << data->z << "), "
         //   << "Heading: " << data->heading << std::endl;


        current_data.x = data->z;
        current_data.y = data->x;
        current_data.h = data->heading;
        
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;

        current_data.t = elapsed.count();
        // Optionally, sleep to avoid excessive polling
        usleep(3000);  // 1 ms
        
        
        //nav_flag = true;
        //if (elapsed.count() > 600) { running = false; }

    }

    // Cleanup
    munmap(shm_addr, sizeof(OptiTrackData));
    close(shm_fd);
    sem_close(sem);

    return;
}

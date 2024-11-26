
import threading
import time
import struct
from gpiozero import PWMOutputDevice
from NatNetClient import NatNetClient
from util import quaternion_to_euler
import numpy as np
import keyboard

motor = PWMOutputDevice(12, frequency=50)  # Motor
steering = PWMOutputDevice(13, frequency=240)  # Steering
# Pre-allocate memory: position (3), rotation (4), timestamp (1)
data = np.zeros((439200, 8))# Shape: (updates, features)
update_index = 0
throttle_input = 0.1
steering_input = 0.4
navigation_enabled = False
lock = threading.Lock()
shutdown_flag = False
loop_counter = 0

# Example waypoints
waypoints1 = [
    [3.0, 3.0],  # Starting position
    [-3.0, 3.0],
    [-3.0, -3.0],
    [3.0, -3.0]
]

waypoints = [
        [.205,-6.516],
    [-3.558,-5.775],
    [-1.095,-.26],
    [-3.526,1.991],
    [-3.424,2.832],
    [-2.270,5.314],
    [1.131,5.394],
    [4.112,4.590],
    [4.25,1.452],
    [2.238,-1.52],
    [4.224,-4.417]
    ]


current_waypoint_index = 0

def toggle_navigation():
    """
    Toggles the navigation state when the spacebar is pressed.
    If navigation is enabled, it will disable navigation and stop the car.
    If navigation is disabled, it will re-enable navigation.
    """
    global navigation_enabled, throttle_input, steering_input

    if navigation_enabled:
        # Disable navigation and stop the car
        print("Spacebar pressed: Disabling navigation and stopping the car.")
        navigation_enabled = False
        throttle_input = 0.08  # Set throttle to minimum
        steering_input = 0.4  # Set steering to neutral
    else:
        # Re-enable navigation
        print("Spacebar pressed: Re-enabling navigation.")
        navigation_enabled = True


def increase_throttle():
    """
    Increases the throttle when the 'up' arrow key is pressed.
    """
    global throttle_input
    throttle_input = min(throttle_input + 0.01, .12)  # Max throttle: .12f
    print(f"Increasing throttle: {throttle_input:.2f}")

def decrease_throttle():
    """
    Decreases the throttle when the 'down' arrow key is pressed.
    """
    global throttle_input
    throttle_input = max(throttle_input - 0.01, .04)  # Min throttle: 0.04
    print(f"Decreasing throttle: {throttle_input:.2f}")

def turn_left():
    """
    Turns the car left when the 'left' arrow key is pressed.
    """
    global steering_input
    steering_input = max(steering_input - 0.01, .32)  # Full left: .32
    print(f"Turning left: {steering_input:.2f}")

def turn_right():
    """
    Turns the car right when the 'right' arrow key is pressed.
    """
    global steering_input
    steering_input = min(steering_input + 0.01, .48)  # Full right: 1.0
    print(f"Turning right: {steering_input:.2f}")

def setup_keyboard_events():
    """
    Sets up event-driven keyboard handlers.
    """
    # Register the spacebar for stopping navigation
    keyboard.on_press_key("space", lambda _: toggle_navigation())

    # Register arrow keys for throttle and steering adjustments
    keyboard.on_press_key("up", lambda _: increase_throttle())
    keyboard.on_press_key("down", lambda _: decrease_throttle())
    keyboard.on_press_key("left", lambda _: turn_left())
    keyboard.on_press_key("right", lambda _: turn_right())

    print("Keyboard event handlers are set up. Press SPACE to stop navigation.")

def get_current_position_and_heading():
    """
    Extracts the car's current 2D position (z, x) and heading (yaw) from the data array.
    Truncates the values to the first three digits past the decimal.

    Returns:
        tuple: (current_position, current_heading)
    """
    global update_index

    if update_index == 0:
        raise ValueError("No position data available yet.")

    # Extract quaternion (qw, qx, qy, qz) and truncate to 3 decimal places
    qx, qy, qz, qw = np.around(data[update_index - 1, 3:7], decimals=3)

    # Adjust quaternion: flip if the rotation vector is negative
    if qy < 0:
        qw = qw * -1

    # Compute heading (yaw) from quaternion using 2 * arccos(qw)
    heading = 2 * np.arccos(qw)  # Result is in radians
    heading = np.degrees(heading)  # Convert to degrees

    # Normalize heading to [0, 360) and adjust for x-positive alignment
    #if heading > 180:
     #   heading -= 360  # Normalize to [-180, 180]

    # Extract position (z, x) and truncate to 3 decimal places
    current_position = np.around([data[update_index - 1, 0], data[update_index - 1, 2]], decimals=3)  # [x, z]

    return current_position, heading


def compute_steering(current_position, current_heading):
    """
    Computes the steering adjustment based on the car's position and heading
    relative to the next waypoint.
    
    Args:
        current_position (list): The car's current position [z, x].
        current_heading (float): The car's current heading in degrees.

    Returns:
        float: Steering value scaled between -1.0 (full left) and 1.0 (full right).
    """
    global current_waypoint_index, loop_counter
    
    # Check if all waypoints are reached
    if current_waypoint_index >= len(waypoints):
        print("All waypoints reached! Resetting.")
        current_waypoint_index = 0
    

    # Get the target waypoint
    target_waypoint = waypoints[current_waypoint_index]

    # Compute the vector to the waypoint
    dx = target_waypoint[0] - current_position[0]  # x-axis difference
    dz = -(target_waypoint[1] - current_position[1])  # z-axis difference
    distance_to_waypoint = np.hypot(dz, dx)

    # Check if the waypoint is reached
    if distance_to_waypoint < 0.15:  # Consider waypoint reached if within 15 cm
        print(f"Waypoint {current_waypoint_index} reached!")
        current_waypoint_index += 1
        if current_waypoint_index >= len(waypoints):
            print("All waypoints reached!")
            return 0.0  # Neutral steering
        target_waypoint = waypoints[current_waypoint_index]  # Update to next waypoint

    # Compute the desired heading (angle to waypoint) in degrees
    desired_heading = np.degrees(np.arctan2(dz, dx))

    # Compute steering adjustment
    heading_error = desired_heading - current_heading
    heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]

    # Proportional steering control (scale error to [-1, 1])
    output = np.clip(heading_error / 75, -1.0, 1.0)

    steering = .4 + output*.08

    if abs(0.4 - steering) > .04:
        # Control PWM devices
        throttle_input = .095
        
    if abs(0.4 - steering) <= .04:
        throttle_input = 0.1

    if loop_counter % 20 == 0:
        print(f"Current Pos: {current_position}, Target: {target_waypoint}, desired: {desired_heading}"
              f"Heading Error: {heading_error:.2f}, Steering: {steering:.2f}")

    return steering


def initialize_position(threshold=0.1, timeout=30):
    """
    Waits for the car's position to change by at least `threshold` units
    compared to the first recorded position, within `timeout` seconds.
    """
    start_time = time.time()
    reference_position = None


    global steering_input, throttle_input, motor, steering

    steering.value = .4
    motor.value = .1

    while time.time() - start_time < timeout:
        if update_index >= 1:  # Ensure at least one position is recorded
            if reference_position is None:
                with lock:
                    # Set the first position as the reference
                    reference_position = data[update_index - 1, 0:3]
                print(f"Reference position set to: {reference_position}")
            with lock:
                # Compare the current position to the reference position
                current_position = data[update_index - 1, 0:3]
            displacement = np.linalg.norm(current_position - reference_position)

            if displacement >= threshold and update_index>6:
                print(f"Vehicle moved {displacement:.3f} units. Initialization complete.")
                navigation_enabled = True
                return True

        time.sleep(0.05)  # Check every 50ms

    print("Initialization failed: No significant movement detected.")
    return False

def pwm_control(throttle, steer_cmd):
    global motor, steering

    if abs(0.4 - steer_cmd) > .04:
        # Control PWM devices

        motor.value = .099

    if abs(0.4 - steer_cmd) <= .04:
        motor.value = 0.1

    if abs(steer_cmd - steering.value) > 0.001:  # Avoid tiny adjustments
        steering.value = steer_cmd
    
# Data Streaming Loop
def data_streaming_loop():
    positions = {}
    rotations = {}

    # Define the NatNet client
    clientAddress = "192.168.1.50"
    optitrackServerAddress = "192.168.1.10"
    robot_id = 404

    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(False)

    # Callback for NatNet data
    def receive_rigid_body_frame(id, position, rotation_quaternion):
        global update_index, shutdown_flag
        if id == robot_id and shutdown_flag == False:
            with lock:
                data[update_index, 0:3] = position
                data[update_index, 3:7] = rotation_quaternion
                data[update_index, 7] = time.time()  # Timestamp
                update_index += 1
                if update_index >= 439200:
                    shutdown_flag = True


    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start the NatNet client
    is_running = streaming_client.run()

    try:
        while not shutdown_flag and is_running:
            time.sleep(0.05) 
    finally:
        streaming_client.shutdown()

def check_for_kick():
    """
    Checks if the car is stalled and applies a brief throttle "kick" if needed.
    """
    global data, update_index, throttle_input, loop_counter, motor

    if update_index < 120:
        # Not enough data yet to make a decision
        return

    # Run every 20th loop cycle
    if loop_counter % 5 != 0:
        return

    # Compare current position with position 120 updates back
    current_position = np.array(data[update_index - 1, 0:3])
    previous_position = np.array(data[update_index - 30, 0:3])

    # Compute displacement
    displacement = np.linalg.norm(current_position - previous_position)

    if displacement < 0.05:  # Threshold for detecting a stall
        print(f"Stall detected! Displacement: {displacement:.3f}. Applying kick.")
        # Apply a brief throttle kick
        motor.value = 0.1
        time.sleep(0.25)  # Kick duration
        motor.value = throttle_input  # Return to normal throttle
    else:
        print(f"Car is moving. Displacement: {displacement:.3f}.")



if __name__ == "__main__":
    try:
        # Create threads for each loop
        data_thread = threading.Thread(target=data_streaming_loop)
        data_thread.start()

        while update_index < 1:
            print("Waiting for position data...")
            time.sleep(0.1)


        print("Initializing position...")
        if not initialize_position():
            print("Initialization failed. Exiting.")
            exit(1)

        setup_keyboard_events()

        # Keep the main thread alive
        while not shutdown_flag:
            # get_current_position_and_heading()
            if navigation_enabled == True
                current_position, current_heading = get_current_position_and_heading()
                steering_input = compute_steering(current_position, current_heading)
            pwm_control(throttle_input, steering_input)
            check_for_kick()
            loop_counter += 1
            time.sleep(0.04)

        data_thread.join()
        
        print("All threads exited. Program terminated.")

    except KeyboardInterrupt:
        shutdown_flag = True
        print("KeyboardInterrupt detected. Shutting down...")
        motor.close()
        steering.close()
        data_thread.join()

        print("All threads exited. Program terminated.")
  
 


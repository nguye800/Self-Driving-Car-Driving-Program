import time
from automatic_drive import config

from automatic_drive.disparity import get_obstacle_readings_from_stereo, rpi_camera
from automatic_drive.homing_controller import HomingController
from initialization.init import get_wheel_motors, get_distance_sensors, check_collision, SonarWrapper
from manual_drive.dc_motor import Motor

BACKUP_STEPS = 15

def auto_drive(left_motor: Motor, right_motor: Motor, sonar_left: SonarWrapper, sonar_right: SonarWrapper):
    # Initial setup
    camL = rpi_camera(1)
    camR = rpi_camera(0)
    
    
    # Wait for cameras + sonar to initialize
    time.sleep(3)
    
    controller = HomingController()
    
    reached_target = False
    last_reading_time = time.time()
    backup_steps_remaining = 0
    
    while not reached_target:
        # If we are in backup mode, just go straight backward for a few steps
        if backup_steps_remaining > 0:
            # Back up at half of max linear speed (tune as you like)
            back_speed = 0.5 * controller.maxLinearVel
            
            left_motor.backward(back_speed)
            right_motor.backward(back_speed)
            backup_steps_remaining -= 1
            # Skip normal homing logic this step
            continue
        
        # Calculate goal vector
        goalDist, goalAngle = 5, 0 # replace with bluetooth logic
        
        # Short-range sonar check: if too close, trigger backup mode
        if check_collision([sonar_left, sonar_right]):
            backup_steps_remaining = BACKUP_STEPS
            # First immediate backup command
            back_speed = 0.5 * controller.maxLinearVel
            left_motor.backward(back_speed)
            right_motor.backward(back_speed)
            continue  # don't run stereo/homing logic this step
        
        # Compute stereo only every 15 steps
        if time.time() - last_reading_time > 0.5:
            centerReading, leftReading, rightReading = get_obstacle_readings_from_stereo(camL, camR, visualize=False, save_disp=False)
            last_reading_time = time.time()
        else:
            # reuse last stereo readings for control
            centerReading, leftReading, rightReading = config._last_readings
        
        # Compute motor velocities using the homing algorithm
        leftVel, rightVel = controller.compute_velocities(goalDist, goalAngle, 
            centerReading, leftReading, rightReading, debug=False)
        
        # Apply velocities
        if leftVel >= 0:   
            left_motor.forward(leftVel)
        else:
            left_motor.backward(abs(leftVel))
        
        if rightVel >= 0:
            right_motor.forward(rightVel)
        else:
            right_motor.backward(abs(rightVel))
        
        # Check if target reached switch to manual mode
        if goalDist < 1.5:
            reached_target = True
            left_motor.stop(reached_target)
            right_motor.stop(reached_target)
            return 0
        
    return 1

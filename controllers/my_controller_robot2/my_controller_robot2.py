
from controller import Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())


wheels = [robot.getDevice(name) for name in ["wheel1", "wheel2", "wheel3", "wheel4"]]
for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0)


encoders = [wheel.getPositionSensor() for wheel in wheels]
for encoder in encoders:
    encoder.enable(timestep)

start_position = encoders[0].getValue()

armMotors = [robot.getDevice(f"arm{i}") for i in range(1, 6)]
for motor in armMotors:
    motor.setVelocity(1.5)


armPositionSensors = [robot.getDevice(f"arm{i}sensor") for i in range(1, 6)]
for sensor in armPositionSensors:
    sensor.enable(timestep)


finger1 = robot.getDevice("finger::left")
finger2 = robot.getDevice("finger::right")
finger1.setVelocity(1.5)
finger2.setVelocity(1.5)
fingerMinPosition = finger1.getMinPosition()
fingerMaxPosition = finger1.getMaxPosition()

camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)

camera2 = robot.getDevice("camera2")
if camera2:
    camera2.enable(timestep)

sensors = robot.getDevice("distance_sensor")
if sensors :
   sensors.enable(timestep)




receiver = robot.getDevice("receiver")
receiver.setChannel(1)
receiver.enable(timestep)



detected_colors_array = []


initial_positions = [encoder.getValue() for encoder in encoders]
distance_reached = False


def rotate_robot(direction):
 
    if direction == "right":
        velocities = [-rotation_speed, rotation_speed, -rotation_speed, rotation_speed]
    elif direction == "left":
        velocities = [rotation_speed, -rotation_speed, rotation_speed, -rotation_speed]
    else:
        return

    for wheel, vel in zip(wheels, velocities):
        wheel.setVelocity(vel)

    robot.step(rotation_duration)  # مدة الدوران
    for wheel in wheels:
        wheel.setVelocity(0)


def move_forward(time):
    for wheel in wheels:
        wheel.setVelocity(7.0)
    robot.step(time * timestep)

def move_backward(time):
    for wheel in wheels:
        wheel.setVelocity(-7.0)
    robot.step(time * timestep)

def halt():
    for wheel in wheels:
        wheel.setVelocity(0.0)

def turn(time, direction='left'):
    if direction == 'left':
        wheels[0].setVelocity(-7.0)
        wheels[1].setVelocity(7.0)
        wheels[2].setVelocity(-7.0)
        wheels[3].setVelocity(7.0)
    elif direction == 'right':
        wheels[0].setVelocity(7.0)
        wheels[1].setVelocity(-7.0)
        wheels[2].setVelocity(7.0)
        wheels[3].setVelocity(-7.0)
    robot.step(time * timestep)
    halt()


def move_forward(robot, wheels, time):
    """
    Move the robot forward for a specified time.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param time: Time to move forward
    """
    for wheel in wheels:
        wheel.setVelocity(7.0)
    robot.step(time * timestep)

def move_backward(robot, wheels, time):
    """
    Move the robot backward for a specified time.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param time: Time to move backward
    """
    for wheel in wheels:
        wheel.setVelocity(-7.0)  # Negative velocity for backward movement
    robot.step(time * timestep)
    halt_robot(wheels)  # Stop the robot after the movement




def move_to_position(robot, wheels, encoders, desired_position):
    """
    Move the robot to a desired position using encoders.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param encoders: List of wheel encoders
    :param desired_position: The target encoder position to return to
    """
    print(f"Returning to desired position: {desired_position}")

    # Calculate the current encoder positions
    current_positions = [encoder.getValue() for encoder in encoders]

    # Calculate the movement needed to reach the desired position
    distances_to_target = [desired_position - current for current in current_positions]

    # Determine the direction of motion (-1 for backward, 1 for forward)
    directions = [1 if distance > 0 else -1 for distance in distances_to_target]

    # Set velocity for the wheels based on the required direction
    for wheel, direction in zip(wheels, directions):
        wheel.setVelocity(5.0 * direction)

    # Continue until all wheels reach the desired position
    while robot.step(timestep) != -1:
        # Update the distances to the target position
        distances_to_target = [
            abs(desired_position - encoder.getValue()) for encoder in encoders
        ]

        # Stop if all wheels are near the desired position
        if all(distance < 0.01 for distance in distances_to_target):  # Adjust margin as needed
            halt_robot(wheels)
            print("Robot has reached the desired position.")
            break


def halt_robot(wheels):
    """
    Stop all wheels.
    """
    for wheel in wheels:
        wheel.setVelocity(0.0)


def move_forward_for_distance(robot, wheels, encoders, distance, timestep):
    """
    Move forward a specific distance using encoders.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param encoders: List of wheel encoders
    :param distance: Distance in meters to move forward
    :param timestep: Simulation timestep
    """
    # Set initial encoder positions
    WHEEL_RADIUS = 0.05  # Radius of the wheel in meters (adjust based on your robot)
    WHEEL_CIRCUMFERENCE = 2 * 3.14159 * WHEEL_RADIUS  # Circumference of the wheel

    # Set initial encoder positions
    initial_positions = [encoder.getValue() for encoder in encoders]
    
    # Calculate the target encoder increment for the desired distance
    target_increment = distance / WHEEL_CIRCUMFERENCE  # Number of wheel rotations needed

    # Set target encoder positions
    target_positions = [initial + target_increment for initial in initial_positions]

    while robot.step(timestep) != -1:
        # Set wheel velocity to move forward
        for wheel in wheels:
            wheel.setVelocity(5.0)  # Set a reasonable forward velocity

        # Get current encoder positions
        current_positions = [encoder.getValue() for encoder in encoders]

        # Check if the robot has traveled the desired distance
        distances_covered = [abs(current - initial) for current, initial in zip(current_positions, initial_positions)]
        if all(distance_covered >= target_increment for distance_covered in distances_covered):
            # Stop the robot when the target distance is reached
            for wheel in wheels:
                wheel.setVelocity(0.0)
            print(f"Target distance of {distance} meters reached.")
            break



def move_until_wall(robot, wheels, distance_sensor, threshold_distance, timestep):
    """
    Move the robot forward until the distance sensor detects a wall within the threshold distance.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param distance_sensor: Distance sensor device
    :param threshold_distance: Distance threshold in meters
    :param timestep: Simulation timestep
    """
    # Start moving the robot forward
    for wheel in wheels:
        wheel.setVelocity(7.0)  # Set a forward velocity
    
    print(f"Moving forward until the wall is {threshold_distance} meters away...")
    
    while robot.step(timestep) != -1:
        # Get the current distance from the wall
        sensor_value = distance_sensor.getValue()
        print(f"Distance sensor reading: {sensor_value} meters")
        
        # Stop if the distance sensor detects a wall closer than the threshold
        if sensor_value <= threshold_distance:
            print(f"Wall detected at {sensor_value} meters. Stopping.")
            for wheel in wheels:
                wheel.setVelocity(0.0)  # Stop the robot
            break
            
            
def rotate_robot(robot, wheels, direction, angle, timestep):
    """
    Rotate the robot by a specific angle.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param direction: 'left' or 'right'
    :param angle: Angle to rotate in degrees
    :param timestep: Simulation timestep
    """
    # Adjust timing based on wheel speed and rotation requirements
    rotation_time = int(angle / 90 * 300)  # Adjust timing for 90-degree turns

    if direction == 'left':
        wheels[0].setVelocity(-2.0)
        wheels[1].setVelocity(2.0)
        wheels[2].setVelocity(-2.0)
        wheels[3].setVelocity(2.0)
    elif direction == 'right':
        wheels[0].setVelocity(2.0)
        wheels[1].setVelocity(-2.0)
        wheels[2].setVelocity(2.0)
        wheels[3].setVelocity(-2.0)

    for _ in range(rotation_time):
        robot.step(timestep)
    halt_robot(wheels)



def pick_up_box():
    print("Picking up the box...")
    robot.step(50 * timestep)
    # Position arm to reach down
    armMotors[0].setPosition(0.0)  # Base rotation
    armMotors[1].setPosition(-1.1)  # Shoulder
    armMotors[2].setPosition(-1)  # Elbow
    armMotors[3].setPosition(-1)  # Wrist
    finger1.setPosition(fingerMaxPosition)  # Open claw
    finger2.setPosition(fingerMaxPosition)
    
    robot.step(50 * timestep)
    
    # Close claw to grab the cube
    finger1.setPosition(0)
    finger2.setPosition(0)
    robot.step(50 * timestep)
    
    # Lift the cube
    armMotors[1].setPosition(0.0)  # Shoulder to default
    armMotors[2].setPosition(0.0)  # Elbow to default
    armMotors[3].setPosition(0.0)  # Wrist to default
    robot.step(50 * timestep)


def pick_up_box_from_wall():
    print("Picking up the box from wall...")
    robot.step(50 * timestep)
    armMotors[0].setPosition(0)  # Base rotation
    armMotors[1].setPosition(-0.4)  # Shoulder
    armMotors[2].setPosition(0)  # Elbow
    armMotors[3].setPosition(-1.7)  # Wrist
    finger1.setPosition(fingerMaxPosition)  # Ensure claw is open
    finger2.setPosition(fingerMaxPosition)
    robot.step(50 * timestep)
    
    robot.step(50 * timestep)
    
    # Close claw to grab the cube
    finger1.setPosition(0)
    finger2.setPosition(0)
    robot.step(50 * timestep)
    
    # Lift the cube
    armMotors[1].setPosition(0.0)  # Shoulder to default
    armMotors[2].setPosition(0.0)  # Elbow to default
    armMotors[3].setPosition(0.0)  # Wrist to default
    robot.step(50 * timestep)
    
    
def place_box_on_robot():
    print("Placing the box on the robot...")
    # Position arm to place the cube on the wall
    armMotors[0].setPosition(-2.9495)  # Base rotation to face the wall
    armMotors[1].setPosition(-0.6)  # Shoulder position
    armMotors[2].setPosition(-1.0)  # Elbow
    armMotors[3].setPosition(-1.2)  # Wrist
    robot.step(250 * timestep)
    
    # Open claw to release the cube
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(50 * timestep)
    

def place_box_on_wall():
    print("Placing the box on the wall...")
   
    # Open claw to release the cube
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(50 * timestep)

def place_box_on_color():
    print("Placing the box on the target color...")
    # Position arm to place the cube on the wall
    armMotors[0].setPosition(-1)  # Base rotation to face the wall
    armMotors[1].setPosition(-0.6)  # Shoulder position
    armMotors[2].setPosition(-1.0)  # Elbow
    armMotors[3].setPosition(-1.2)  # Wrist
    robot.step(250 * timestep)
    
    # Open claw to release the cube
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(50 * timestep)
       
def pick_box_from_robot():
    print("Placing the box on the robot...")
    # Position arm to place the cube on the wall
    armMotors[0].setPosition(-2.9495)  # Base rotation to face the wall
    armMotors[1].setPosition(-0.6)  # Shoulder position
    armMotors[2].setPosition(-1.0)  # Elbow
    armMotors[3].setPosition(-1.2)  # Wrist
    robot.step(250 * timestep)
    
    # Open claw to release the cube
    finger1.setPosition(0)
    finger2.setPosition(0)
    robot.step(50 * timestep)



def receive_from_robot():
    if receiver.getQueueLength() > 0:
            message = receiver.getData().decode('utf-8')
            receiver.nextPacket()  # Clear the current message from the queue

            # Parse the message
            if message.startswith("cube_placed"):
                _, cube_position = message.split(",")
                cube_position = int(cube_position)
                print(f"Received cube placement at index {cube_position}. Moving to pick it up...")


def process_receiver_message(robot, receiver, timestep):
    """
    Process incoming messages from the receiver and return cube position if found.
    :param robot: Robot object
    :param receiver: Receiver device
    :param timestep: Simulation timestep
    :return: Cube position if message is received; None otherwise
    """
    while robot.step(timestep) != -1:
        if receiver.getQueueLength() > 0:
            message = receiver.getString()  # Use updated method
            receiver.nextPacket()  # Clear the current message from the queue

            # Parse the message
            if message.startswith("cube_placed"):
                _, cube_position = message.split(",")
                cube_position = int(cube_position)
                print(f"Received cube placement at index {cube_position}.")
                return cube_position  # Return the cube's position
    return None


def reset_arm_position():
    print("Resetting arm to initial position...")
    armMotors[0].setPosition(0.0)  # Base rotation
    armMotors[1].setPosition(0.0)  # Shoulder
    armMotors[2].setPosition(0.0)  # Elbow
    armMotors[3].setPosition(0.0)  # Wrist
    finger1.setPosition(fingerMaxPosition)  # Ensure claw is open
    finger2.setPosition(fingerMaxPosition)
    robot.step(50 * timestep)


def rotate_arm_position():
    print("Resetting arm to initial position...")
    armMotors[0].setPosition(0.0)  # Base rotation
    armMotors[1].setPosition(0.0)  # Shoulder position
    armMotors[2].setPosition(0)  # Elbow
    armMotors[3].setPosition(-1.5)  # Wrist
    
    robot.step(50 * timestep)




def perform_scan( distance_to_move=25):
    """
    Move the robot forward for 5 meters while scanning colors.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param encoders: List of wheel encoders
    :param camera: Camera device
    :param detected_colors: List of detected colors
    :param distance_to_move: Distance in meters to move forward
    """
    print("Starting the scan and movement task...")

    # Get the timestep from the robot
    timestep = int(robot.getBasicTimeStep())
    if timestep <= 0:
        raise ValueError("Invalid timestep value. Check the robot configuration.")

    # Calculate the target encoder position (assuming encoder unit corresponds to meters)
    initial_positions = [encoder.getValue() for encoder in encoders]
    target_positions = [initial + distance_to_move for initial in initial_positions]

    while robot.step(timestep) != -1:
        # Move the robot forward
        for wheel in wheels:
            wheel.setVelocity(5.0)  # Adjust speed as needed

        # Read current encoder positions
        current_positions = [encoder.getValue() for encoder in encoders]

        # Check if the target distance has been reached
        distances = [
            abs(current - initial) for current, initial in zip(current_positions, initial_positions)
        ]
        if all(distance >= distance_to_move for distance in distances):
            print("Target distance reached. Stopping the robot.")
            halt()
            break

        # Perform color scanning
        if camera:
            image = camera.getImage()
            width, height = camera.getWidth(), camera.getHeight()
            if image:
                x, y = width // 2, height // 2
                red = camera.imageGetRed(image, width, x, y)
                green = camera.imageGetGreen(image, width, x, y)
                blue = camera.imageGetBlue(image, width, x, y)

                # Detect color
                detected_color = None
                if red > 200 and green < 100 and blue < 100:
                    detected_color = "Red"
                elif green > 200 and red < 100 and blue < 100:
                    detected_color = "Green"
                elif blue > 200 and red < 100 and green < 100:
                    detected_color = "Blue"
                elif red > 200 and green > 200 and blue < 100:
                    detected_color = "Yellow"

                # Add the color to the list if not already detected
                if detected_color and detected_color not in detected_colors_array:
                    detected_colors_array.append(detected_color)
                    print(f"Detected color: {detected_color}")



def scan_area(camera2):
    """
    Scan the current area for the correct cube.
    :param camera: Camera device
    :return: Detected color, if found; otherwise, None
    """
    if camera2:
        image = camera2.getImage()
        width, height = camera2.getWidth(), camera2.getHeight()
        if image:
            x, y = width // 2, height // 2
            red = camera2.imageGetRed(image, width, x, y)
            green = camera2.imageGetGreen(image, width, x, y)
            blue = camera2.imageGetBlue(image, width, x, y)

            # Detect color
            detected_color = None
            if red > 200 and green < 100 and blue < 100:
                detected_color = "Red"
            elif green > 200 and red < 100 and blue < 100:
                detected_color = "Green"
            elif blue > 200 and red < 100 and green < 100:
                detected_color = "Blue"
            elif red > 200 and green > 200 and blue < 100:
                detected_color = "Yellow"

            if detected_color:
                print(f"Detected color: {detected_color}")
            return detected_color
    return None


def navigate_to_all_cubes(robot, wheels, encoders, camera2, detected_colors, color_order, timestep):
    """
    Navigate to all cubes based on the color order.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param encoders: List of wheel encoders
    :param camera: Camera device
    :param detected_colors: List to store detected colors
    :param color_order: List of colors to search for in order
    :param timestep: Simulation timestep
    """
    for target_color in color_order:
        print(f"Searching for {target_color} cube...")
        found_cube = False

        while not found_cube:
            # Area 1: Move forward and scan
            move_forward_for_distance(robot, wheels, encoders, 15.0, timestep)
            rotate_robot(robot, wheels, 'left', 120, timestep)
            detected_color = scan_area(camera2)
            if detected_color == target_color:
                print(f"Found {target_color} in Area 1!")
                detected_colors.append(target_color)
                
                pick_up_box()
                #place_box_on_robot()
                #reset_arm_position()
                #pick_box_from_robot()
                rotate_arm_position()
                rotate_robot(robot, wheels, 'right', 120, timestep)
                move_forward_for_distance(robot, wheels, encoders, 35, timestep)
                place_box_on_wall()
                reset_arm_position()
                #rotate_robot(robot, wheels, 'right', 120, timestep)
                
                print("Returning to start position...")
                move_to_position(robot, wheels, encoders, start_position)
                found_cube = True
                break

            rotate_robot(robot, wheels, 'right', 240, timestep)
            detected_color = scan_area(camera2)
            if detected_color == target_color:
                print(f"Found {target_color} in Area 2!")
                detected_colors.append(target_color)
                
                pick_up_box()
                #place_box_on_robot()
                #reset_arm_position()
                #pick_box_from_robot()
                rotate_arm_position()
                rotate_robot(robot, wheels, 'left', 120, timestep)
                move_forward_for_distance(robot, wheels, encoders, 35, timestep)
                place_box_on_wall()
                reset_arm_position()
                
                
                print("Returning to start position...")
                move_to_position(robot, wheels, encoders, start_position)
                found_cube = True
                break

            rotate_robot(robot, wheels, 'left', 120, timestep)
            

            # Area 3: Move forward again and scan
            move_forward_for_distance(robot, wheels, encoders, 20, timestep)
            rotate_robot(robot, wheels, 'left', 120, timestep)
            detected_color = scan_area(camera2)
            if detected_color == target_color:
                print(f"Found {target_color} in Area 3!")
                detected_colors.append(target_color)
                
                
                pick_up_box()
                #place_box_on_robot()
                #reset_arm_position()
                #pick_box_from_robot()
                rotate_arm_position()
                rotate_robot(robot, wheels, 'right', 120, timestep)
                move_forward_for_distance(robot, wheels, encoders, 15, timestep)
                place_box_on_wall()
                reset_arm_position()
                
                
                print("Returning to start position...")
                move_to_position(robot, wheels, encoders, start_position)
                found_cube = True
                break

            # Area 4: Rotate -90 degrees and scan
            rotate_robot(robot, wheels, 'right', 240, timestep)
            detected_color = scan_area(camera2)
            if detected_color == target_color:
                print(f"Found {target_color} in Area 4!")
                detected_colors.append(target_color)
                
                
                pick_up_box()
                #place_box_on_robot()
                #reset_arm_position()
                #pick_box_from_robot()
                rotate_arm_position()
                rotate_robot(robot, wheels, 'left', 120, timestep)
                move_forward_for_distance(robot, wheels, encoders, 15, timestep)
                place_box_on_wall()
                reset_arm_position()
                
                
                print("Returning to start position...")
                move_to_position(robot, wheels, encoders, start_position)
                found_cube = True
                break

            # Rotate back to original position and return to initial position
            rotate_robot(robot, wheels, 'left', 120, timestep)
            move_to_position(robot, wheels, encoders, start_position)
            

        print(f"{target_color} cube located successfully!")
    print("Completed search for all cubes.")



def navigate_to_cube_on_wall(robot, wheels, encoders, camera2, detected_colors, color_order, timestep):
    """
    Navigate to all cubes based on the color order.
    :param robot: Robot object
    :param wheels: List of wheel devices
    :param encoders: List of wheel encoders
    :param camera: Camera device
    :param detected_colors: List to store detected colors
    :param color_order: List of colors to search for in order
    :param timestep: Simulation timestep
    """
    for target_color in color_order:
        print(f"Searching for {target_color} cube...")
        found_cube = False

        while not found_cube:
            # Area 1: Move forward and scan
            move_forward_for_distance(robot, wheels, encoders, 50.0, timestep)
            detected_color = scan_area(camera2)
            if detected_color == target_color:
                print(f"Found {target_color} in Area 1!")
                detected_colors.append(target_color)
                
                pick_up_box()
                #place_box_on_robot()
                #reset_arm_position()
                #pick_box_from_robot()
                rotate_arm_position()
                
                move_forward_for_distance(robot, wheels, encoders, -50, timestep)
                place_box_on_wall()
                reset_arm_position()
                #rotate_robot(robot, wheels, 'right', 120, timestep)
                
                print("Returning to start position...")
                move_to_position(robot, wheels, encoders, start_position)
                found_cube = True
                break

            

            
            move_to_position(robot, wheels, encoders, start_position)
            

        print(f"{target_color} cube located successfully!")
    print("Completed search for all cubes.")

def search_for_the_right_area(robot, wheels, encoders, camera2,target_color, start_position, timestep):
    """
    Search for the correct area to place the cube and place it in the correct position.
    :param robot: Second robot object
    :param wheels: List of wheel devices for the second robot
    :param encoders: List of wheel encoders
    :param camera: Camera device for scanning
    :param arm: Arm device to lift the cube
    :param gripper: Gripper device to grab/release the cube
    :param target_color: Color of the cube to place
    :param start_position: Starting position of the robot
    :param timestep: Simulation timestep
    """
    print(f"Searching for the correct area to place {target_color} cube...")
    found_area = False

    while not found_area:
        # Area 1: Move forward and scan
        move_forward_for_distance(robot, wheels, encoders, 15.0, timestep)
        rotate_robot(robot, wheels, 'left', 120, timestep)
        detected_color = scan_area(camera2)
        if detected_color == target_color:
            print(f"Found the correct area for {target_color} (Area 1).")
            place_box_on_color()
            reset_arm_position()
            rotate_robot(robot, wheels, 'right', 120, timestep)
            print("Returning to start position...")
            move_to_position(robot, wheels, encoders, start_position)
            found_area = True
            break

        # Area 2: Rotate to the right and scan
        rotate_robot(robot, wheels, 'right', 240, timestep)
        detected_color = scan_area(camera2)
        if detected_color == target_color:
            print(f"Found the correct area for {target_color} (Area 2).")
            place_box_on_color()
            reset_arm_position()
            rotate_robot(robot, wheels, 'left', 120, timestep)
            print("Returning to start position...")
            move_to_position(robot, wheels, encoders, start_position)
            found_area = True
            break

        # Area 3: Move forward again and scan
        rotate_robot(robot, wheels, 'left', 120, timestep)
        move_forward_for_distance(robot, wheels, encoders, 20.0, timestep)
        rotate_robot(robot, wheels, 'left', 120, timestep)
        detected_color = scan_area(camera2)
        if detected_color == target_color:
            print(f"Found the correct area for {target_color} (Area 3).")
            place_box_on_color()
            reset_arm_position()
            rotate_robot(robot, wheels, 'right', 120, timestep)
            print("Returning to start position...")
            move_to_position(robot, wheels, encoders, start_position)
            found_area = True
            break

        # Area 4: Rotate -90 degrees and scan
        rotate_robot(robot, wheels, 'right', 240, timestep)
        detected_color = scan_area(camera2)
        if detected_color == target_color:
            print(f"Found the correct area for {target_color} (Area 4).")
            place_box_on_color()
            reset_arm_position()
            rotate_robot(robot, wheels, 'left', 120, timestep)
            print("Returning to start position...")
            move_to_position(robot, wheels, encoders, start_position)
            found_area = True
            break

        # Return to the original position
        rotate_robot(robot, wheels, 'left', 120, timestep)
        move_to_position(robot, wheels, encoders, start_position, timestep)

    print(f"Successfully placed the {target_color} cube.")

def pick_cubes(robot, wheels, encoders, receiver,detected_colors, color_order, timestep):
    """
    Scan the wall for each cube color and pick it up based on received messages.
    :param robot: Second robot object
    :param wheels: List of wheel devices for the second robot
    :param encoders: List of wheel encoders
    :param arm: Arm device to lift the cube
    :param gripper: Gripper device to grab the cube
    :param receiver: Receiver device to listen for messages
    :param color_order: List of colors to search for in order
    :param timestep: Simulation timestep
    """
    for target_color in color_order:
        
        # Move to the cube's position
        move_forward_for_distance(robot, wheels, encoders, 46, timestep)
        print(f"Waiting for {target_color} cube to be placed on the wall...")
        found_cube = False
        cube_placed = False

        while not found_cube:
            cube_position = process_receiver_message(robot, receiver, timestep)

            if cube_position is not None:
                print(f"Moving to pick up {target_color} cube at index {cube_position}...")

                

                # Pick up the cube
                pick_up_box_from_wall()

                print(f"{target_color} cube picked up by the second robot.")
                move_to_position(robot, wheels, encoders, start_position)
                search_for_the_right_area(robot, wheels, encoders, camera2,target_color, start_position, timestep)
    
                found_cube = True

def perform_task():
    print("Starting the task...")
    perform_scan()
    print("Detected colors array during the scan:", detected_colors_array)
    move_to_position(robot, wheels, encoders, start_position)
    color_order=detected_colors_array
    #move_forward_for_distance(robot, wheels, encoders, 0.5, timestep)
    
    detected_colors = []
    # Start navigation
    #navigate_to_cube_on_wall(robot, wheels, encoders, camera2, detected_colors, color_order, timestep)
    pick_cubes(robot, wheels, encoders, receiver,detected_colors, color_order, timestep)


def main():
    perform_task()

if __name__ == "__main__":
    main()            
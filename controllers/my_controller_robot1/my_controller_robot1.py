from controller import Robot

def main():
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
    fingerMaxPosition = finger1.getMaxPosition()

    camera = robot.getDevice("camera")
    if camera:
        camera.enable(timestep)

    camera2 = robot.getDevice("camera2")
    if camera2:
        camera2.enable(timestep)

    emitter = robot.getDevice("emitter")
    emitter.setChannel(1)

    detected_colors_array = []

    def halt_robot(wheels):
        for wheel in wheels:
            wheel.setVelocity(0.0)

    def move_forward_for_distance(robot, wheels, encoders, distance, timestep):
        WHEEL_RADIUS = 0.05
        WHEEL_CIRCUMFERENCE = 2 * 3.14159 * WHEEL_RADIUS
        initial_positions = [encoder.getValue() for encoder in encoders]
        target_increment = distance / WHEEL_CIRCUMFERENCE
        target_positions = [initial + target_increment for initial in initial_positions]

        while robot.step(timestep) != -1:
            for wheel in wheels:
                wheel.setVelocity(5.0)
            current_positions = [encoder.getValue() for encoder in encoders]
            distances_covered = [abs(current - initial) for current, initial in zip(current_positions, initial_positions)]
            if all(distance_covered >= target_increment for distance_covered in distances_covered):
                for wheel in wheels:
                    wheel.setVelocity(0.0)
                break

    def move_to_position(robot, wheels, encoders, desired_position):
        current_positions = [encoder.getValue() for encoder in encoders]
        distances_to_target = [desired_position - current for current in current_positions]
        directions = [1 if distance > 0 else -1 for distance in distances_to_target]

        for wheel, direction in zip(wheels, directions):
            wheel.setVelocity(5.0 * direction)

        while robot.step(timestep) != -1:
            distances_to_target = [abs(desired_position - encoder.getValue()) for encoder in encoders]
            if all(distance < 0.01 for distance in distances_to_target):
                halt_robot(wheels)
                break
    
    def rotate_robot(robot, wheels, direction, angle, timestep):
        rotation_time = int(angle / 90 * 300)
        if direction == 'left':
            velocities = [-2.0, 2.0, -2.0, 2.0]
        else:
            velocities = [2.0, -2.0, 2.0, -2.0]

        for wheel, velocity in zip(wheels, velocities):
            wheel.setVelocity(velocity)

        for _ in range(rotation_time):
            robot.step(timestep)
        halt_robot(wheels)

    def pick_up_box():
        robot.step(50 * timestep)
        armMotors[0].setPosition(0.0)
        armMotors[1].setPosition(-1.1)
        armMotors[2].setPosition(-1)
        armMotors[3].setPosition(-1)
        finger1.setPosition(fingerMaxPosition)
        finger2.setPosition(fingerMaxPosition)
        robot.step(50 * timestep)
        finger1.setPosition(0)
        finger2.setPosition(0)
        robot.step(50 * timestep)
        armMotors[1].setPosition(0.0)
        armMotors[2].setPosition(0.0)
        armMotors[3].setPosition(0.0)
        robot.step(50 * timestep)

    def place_box_on_wall():
        armMotors[0].setPosition(0.0)
        armMotors[1].setPosition(0)
        armMotors[2].setPosition(0)
        armMotors[3].setPosition(-1.5)
        robot.step(50 * timestep)
        finger1.setPosition(fingerMaxPosition)
        finger2.setPosition(fingerMaxPosition)
        robot.step(50 * timestep)
        emitter.send("cube_placed,0".encode('utf-8'))

    def reset_arm_position():
        armMotors[0].setPosition(0.0)
        armMotors[1].setPosition(0.0)
        armMotors[2].setPosition(0.0)
        armMotors[3].setPosition(0.0)
        finger1.setPosition(fingerMaxPosition)
        finger2.setPosition(fingerMaxPosition)
        robot.step(50 * timestep)

    def rotate_arm_position():
        armMotors[0].setPosition(0.0)
        armMotors[1].setPosition(0.0)
        armMotors[2].setPosition(0)
        armMotors[3].setPosition(-1.5)
        robot.step(50 * timestep)

    def scan_area(camera2):
        if camera2:
            image = camera2.getImage()
            if image:
                width, height = camera2.getWidth(), camera2.getHeight()
                x, y = width // 2, height // 2
                red = camera2.imageGetRed(image, width, x, y)
                green = camera2.imageGetGreen(image, width, x, y)
                blue = camera2.imageGetBlue(image, width, x, y)

                if red > 200 and green < 100 and blue < 100:
                    return "Red"
                elif green > 200 and red < 100 and blue < 100:
                    return "Green"
                elif blue > 200 and red < 100 and green < 100:
                    return "Blue"
                elif red > 200 and green > 200 and blue < 100:
                    return "Yellow"
        return None

    def perform_scan(distance_to_move=25):
        initial_positions = [encoder.getValue() for encoder in encoders]
        target_positions = [initial + distance_to_move for initial in initial_positions]

        while robot.step(timestep) != -1:
            for wheel in wheels:
                wheel.setVelocity(5.0)

            current_positions = [encoder.getValue() for encoder in encoders]
            distances = [abs(current - initial) for current, initial in zip(current_positions, initial_positions)]
            
            if all(distance >= distance_to_move for distance in distances):
                halt_robot(wheels)
                break

            if camera:
                image = camera.getImage()
                width, height = camera.getWidth(), camera.getHeight()
                if image:
                    x, y = width // 2, height // 2
                    red = camera.imageGetRed(image, width, x, y)
                    green = camera.imageGetGreen(image, width, x, y)
                    blue = camera.imageGetBlue(image, width, x, y)

                    detected_color = None
                    if red > 200 and green < 100 and blue < 100:
                        detected_color = "Red"
                    elif green > 200 and red < 100 and blue < 100:
                        detected_color = "Green"
                    elif blue > 200 and red < 100 and green < 100:
                        detected_color = "Blue"
                    elif red > 200 and green > 200 and blue < 100:
                        detected_color = "Yellow"

                    if detected_color and detected_color not in detected_colors_array:
                        detected_colors_array.append(detected_color)

    def navigate_to_all_cubes(robot, wheels, encoders, camera2, detected_colors, color_order, timestep):
        for target_color in color_order:
            found_cube = False

            while not found_cube:
                move_forward_for_distance(robot, wheels, encoders, 15.0, timestep)
                rotate_robot(robot, wheels, 'left', 120, timestep)
                detected_color = scan_area(camera2)
                if detected_color == target_color:
                    detected_colors.append(target_color)
                    pick_up_box()
                    rotate_arm_position()
                    rotate_robot(robot, wheels, 'right', 120, timestep)
                    move_forward_for_distance(robot, wheels, encoders, 35, timestep)
                    place_box_on_wall()
                    reset_arm_position()
                    move_to_position(robot, wheels, encoders, start_position)
                    found_cube = True
                    break

                rotate_robot(robot, wheels, 'right', 240, timestep)
                detected_color = scan_area(camera2)
                if detected_color == target_color:
                    detected_colors.append(target_color)
                    pick_up_box()
                    rotate_arm_position()
                    rotate_robot(robot, wheels, 'left', 120, timestep)
                    move_forward_for_distance(robot, wheels, encoders, 35, timestep)
                    place_box_on_wall()
                    reset_arm_position()
                    move_to_position(robot, wheels, encoders, start_position)
                    found_cube = True
                    break

                rotate_robot(robot, wheels, 'left', 120, timestep)
                move_forward_for_distance(robot, wheels, encoders, 20, timestep)
                rotate_robot(robot, wheels, 'left', 120, timestep)
                detected_color = scan_area(camera2)
                if detected_color == target_color:
                    detected_colors.append(target_color)
                    pick_up_box()
                    rotate_arm_position()
                    rotate_robot(robot, wheels, 'right', 120, timestep)
                    move_forward_for_distance(robot, wheels, encoders, 15, timestep)
                    place_box_on_wall()
                    reset_arm_position()
                    move_to_position(robot, wheels, encoders, start_position)
                    found_cube = True
                    break

                rotate_robot(robot, wheels, 'right', 240, timestep)
                detected_color = scan_area(camera2)
                if detected_color == target_color:
                    detected_colors.append(target_color)
                    pick_up_box()
                    rotate_arm_position()
                    rotate_robot(robot, wheels, 'left', 120, timestep)
                    move_forward_for_distance(robot, wheels, encoders, 15, timestep)
                    place_box_on_wall()
                    reset_arm_position()
                    move_to_position(robot, wheels, encoders, start_position)
                    found_cube = True
                    break

                rotate_robot(robot, wheels, 'left', 120, timestep)
                move_to_position(robot, wheels, encoders, start_position)

    def perform_task():
        perform_scan()
        move_to_position(robot, wheels, encoders, start_position)
        color_order = detected_colors_array
        detected_colors = []
        navigate_to_all_cubes(robot, wheels, encoders, camera2, detected_colors, color_order, timestep)

    perform_task()

if __name__ == "__main__":
    main()
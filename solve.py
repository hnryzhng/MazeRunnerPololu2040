from pololu_3pi_2040_robot import robot
# from pololu_3pi_2040_robot.extras import editions
import time
import gc

# NOTE: To import pololu_3pi_2040_robot for running MicroPython, had to comment out "import ctypes" in lib/pio_quadrature_counter.py
# may need to import ctypes for using quadrature

# robot components
display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
directions = []
encoders = robot.Encoders()
encoder_count = encoders.get_counts()

button_a = robot.ButtonA()  # Note: It's not safe to use Button B in a multi-core program.

# path history variables
path_history = []   # List to store the steps taken
PATH_HISTORY_FILE_NAME = "path_history.txt"

# constants to tweak
max_speed = 1000    # 600
turn_speed = 900
calibration_speed = 1000
calibration_count = 100
encoder_count_max = 220
encorder_count_forward = 120
sensor_threshold = 150  # Line sensor threshold, lower for low-light conditions



def initialize():
    
    """ Actions upon robot startup """

    display_show('initializing')

    # calibrate sensors (line sensor, encoder)
    line = line_sensors.read_calibrated()[:]
    
    calibrate_motors()    

    display_show('finished initialization!')


def calibrate_motors():

    display_show("Put on line and press A to calibrate")

    while not button_a.check():
        pass

    time.sleep_ms(500)

    motors.set_speeds(calibration_speed, -calibration_speed)
    for i in range(calibration_count/4):
        line_sensors.calibrate()

    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(-calibration_speed, calibration_speed)
    for i in range(calibration_count/2):
        line_sensors.calibrate()

    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(calibration_speed, -calibration_speed)
    for i in range(calibration_count/4):
        line_sensors.calibrate()

    motors.off()


# Initialize PID controller variables for solve()
t1 = 0
t2 = time.ticks_us()
p = 0
line = []
starting = False
run_motors = True
last_update_ms = 0
power_difference = 0
prev_message = None 

def solve():

    display_show("Solving...")

    global p, ir, t1, t2, line, max_speed, run_motors, encoder_count
    
    last_p = 0  
    #run_motors = True
    while run_motors:
        # save a COPY of the line sensor data in a global variable
        # to allow the other thread to read it safely.
        line = line_sensors.read_calibrated()[:]
        
        # PID control logic
        line_sensors.start_read()
        t1 = t2
        t2 = time.ticks_us()
        integral = 0

        # Debug: Print raw sensor readings
        print(f"Raw sensor readings: {line}")
        # display.text(f"L snsrs {line}", 0, 20)
        # display.show()

       # postive p means robot is to left of line
        if line[1] < 700 and line[2] < 700 and line[3] < 700:
            if p < 0:
                l = 0
            else:
                l = 4000
        else:
            # estimate line position
            l = (1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // \
                sum(line)

        p = l - 2000
        d = p - last_p
        integral += p
        last_p = p

        # Debug: Print PID values

        # print(f"PID values -> p: {p}, d: {d}, integral: {integral}")
        # display_show(f"PID values -> p: {p}, d: {d}, integral: {integral}")

        denominator = 10 # decreasing this increases the magnitude of the power_difference - makes turns sharper
        power_difference = (p / denominator + integral / 10000 + d * 3 / 2) 
        
        # Debug: Print power difference
        # print(f"Power difference: {power_difference}")
        # display_show(f"Power difference: {power_difference}")

        # constrain calculated power difference within specified max speed
        if(power_difference > max_speed):
            power_difference = max_speed
        if(power_difference < -max_speed):
            power_difference = -max_speed

        # Debug: Print final power difference after limiting
        # print(f"Final power difference: {power_difference}")
        # display_show(f"Final power difference: {power_difference}")

        # Adjust left or right based on calculated power difference
        if(power_difference < 0): 
            motors.set_speeds(max_speed+power_difference, max_speed)
        else:
            motors.set_speeds(max_speed, max_speed-power_difference)
        
        # line = line_sensors.read_calibrated()[:]

        if is_maze_end():
            # update_display("End")
            display_show("End")
            print("Maze END")
            end()

            gc.collect()

            break

        elif (int(line[1]) < 100) and (int(line[2]) < 100) and (int(line[3]) < 100):
            # dead end
            motors.set_speeds(0,0)
            
            dir = "B"
            turn(dir)

            remember(dir)


        elif (int(line[0]) > sensor_threshold) or (int(line[4]) > sensor_threshold):
            # possible valid intersection 
            # left hand or right hand rule?
            # TODO: implement more sophisticated path exploration algo here such as backtracking? 
            # maybe in select_turn() since returns L first, or in get available directions

            motors.set_speeds(0,0)

            directions = get_available_directions()    
            dir = select_turn(directions[0], directions[1], directions[2])
            
            turn(dir)

            remember(dir)

            gc.collect()

def turn(dir: str):

    display.fill(0)
    display.text(dir, 0, 30)
    display.show()

    initial_count = encoders.get_counts()

    if dir == "L":
        motors.set_speeds(-turn_speed, turn_speed)
        # Wait until the robot has turned approximately 90 degrees left
        while not (encoders.get_counts()[0] <= initial_count[0] - int(encoder_count_max) and encoders.get_counts()[1] >= initial_count[1] +int(encoder_count_max)):
            pass
        motors.set_speeds(0, 0)

    elif dir == "R":
        motors.set_speeds(turn_speed, -turn_speed)
        # Wait until the robot has turned approximately 90 degrees right
        while not (encoders.get_counts()[0] >= initial_count[0] + int(encoder_count_max) and encoders.get_counts()[1] <= initial_count[1] - int(encoder_count_max)):
            pass
        motors.set_speeds(0, 0)

    elif dir == "S":
        pass
        # Straight - no specific encoder count adjustment needed here


    elif dir == "B":

        motors.set_speeds(-turn_speed, turn_speed)
        # Wait until the robot has turned approximately 180 degrees
        while not ((encoders.get_counts()[0] <= initial_count[0] - (int(encoder_count_max)*2)) and (encoders.get_counts()[1] >= initial_count[1] + (int(encoder_count_max)*2))):
            pass
        motors.set_speeds(0, 0)

    # else:
    #     display_show("ELSE TURN")
    #     turn("L")
    #     turn("L")

    time.sleep_ms(500)


def select_turn(found_left: bool, found_right: bool, found_straight: bool):
    if found_left:
        return "L"
    elif found_straight:
        return "S"
    elif found_right:
        return "R"
    else:
        return "B"
    
def is_maze_end():
    line = line_sensors.read_calibrated()[:]
    return ((int(line[0]) > 300) and (int(line[1]) > 600) and (int(line[2]) > 600) and (int(line[3]) > 600) and (int(line[4]) > 300))
    

def end():
    motors.off()
    global run_motors
    run_motors = False


def get_available_directions():
    display.fill(0)
    initial_count = encoders.get_counts()

    display_show(f"get a dir")

    try:
        line = line_sensors.read_calibrated()[:]
    except Exception as e:
        return [False, False, False]

    left_dir = False
    right_dir = False
    straight_dir = False

    if int(line[0]) > sensor_threshold:
        left_dir = True
        encoders
                  
    if int(line[4]) > sensor_threshold: 
        right_dir = True

    # keep straight and moving forward    
    # line up with intersection to check for straight line
    motors.set_speeds(500,500)
    while not (encoders.get_counts()[0] >= initial_count[0] + int(encorder_count_forward) and encoders.get_counts()[1] >= initial_count[1] + int(encorder_count_forward)):
        pass
    motors.set_speeds(0, 0)

    line = line_sensors.read_calibrated()[:]

    ### This could cause issues, we are trying to evaluate if the line ahead is straight...ish 
    if (int(line[1]) > sensor_threshold) and (int(line[2]) > sensor_threshold):
        straight_dir = True
    elif (int(line[2]) > sensor_threshold) and (int(line[3]) > sensor_threshold):
        straight_dir = True

    directions = [left_dir, right_dir, straight_dir]

    gc.collect()
    return directions


def remember(direction: str):
    """ Remembers the direction the robot takes. """
    
    global path_history

    path_history.append(direction)  # TODO: right now, path_history is not actually written to file...

    print(f"Path recorded: {direction}")  # Debug print to confirm
    display_show(f"Path recorded: {direction}")

    # log_to_file(direction)  # Log the direction to the file so path history can be accessed even after soft reboot
    
    # TODO: test if path history global var can be updated and retained after each solve() loop
    path_history = path_history # update global path history variable, is this even needed, or does path_history.append() work to update global var path_history?

    # TODO: may need to put log_path_history at end of maze condition within solve()


# Path simplification.  The strategy is that whenever we encounter a
# sequence xBx, we can simplify it by cutting out the dead end.  For
# example, LBL -> S, because a single S bypasses the dead end
# represented by LBL.
def simplify_path():
    """
        Path Simplification Mapping:
        L B L -> S
        R B L -> B
        S B L -> R
    """
    # TODO: should we pass in global var path_history then write to file at end, or reduce and write to file at each call to remember()?
    # if so, may need to rewrite how remember() to how path_history is handled and read from file after each loop?
    path_length = len(path_history)
    
    # validation: simplify path only if second-to-last turn is B
    if (path_length < 3 or path_history[path_length-2]  != 'B'):
        return

    # TODO: if reducing a path chunk that includes S, will need to remember S in path history

    # Approach 1: original implementation with counting angles
    total_angle = 0
    for i in range(1, 3+1):
        current_step = path_history[path_length-i]
        if current_step == 'R':
            total_angle += 90
        elif current_step == 'L':
            total_angle += 270
        elif current_step == 'B':
            total_angle += 180

    # Get angle as number between 0 and 360 degrees
    total_angle = total_angle % 360

    # Replace all of the set of steps with the corresponding single step
    if total_angle == 0:
        path_history[path_length-3] = 'S'
    elif total_angle == 90:
        path_history[path_length-3] = 'R'
    elif total_angle == 180:
        path_history[path_length-3] = 'B'
    elif total_angle == 270:
        path_history[path_length-3] = 'L'
        
    # The path is now 2 steps shorter (3 steps reduced to 1)
    path_length -= 2    # TODO: statement only needed if path_length is a global var


def clear_path_history(filename="path_history.txt"):
    try:
        with open(filename, 'w') as f:
            print("Path history file cleared successfully.")
    except Exception as e:
        print(f"Error clearing file: {e}")

def log_to_file(message, filename="path_history.txt"):
    
    # DEBUG: memory capacity and allocation for file writing
    # Andrew: This really slows things down, and usually gets a lot wrong!
    # kb_free = gc.mem_free() / 1024
    # kb_used = gc.mem_alloc() / 1024
    # kb_total = kb_free + kb_used
    # with open(filename, 'a') as f:
    #     f.write(message + " " + "RAM: "+str(kb_used)+" / "+str(kb_total)+ "\n")
    # gc.collect()

    """ Logs a message (direction) to a file, appending it. """
    try:
        with open(filename, 'a') as f:
            f.write(message + "\n")
        # TODO: add garbage collection?
        # gc.collect()
    except Exception as e:
        print(f"Error logging to file: {e}")

def log_path_history(path_history, filename="path_history.txt"):
    # Write entire path history list to specified file

    try:
        with open(filename, 'w') as f:
            # f.write(message + "\n")
            f.wrirte(path_history)
        # TODO: add garbage collection?
        # gc.collect()
    except Exception as e:
        print(f"Error logging to file: {e}")

def read_path_history(filename="path_history.txt"):
    # Read entire path history list from specified file
    
    path_history = []
    try:
        with open(filename, 'r') as f:
            # lines = f.readlines()
            # path_history = [line.strip() for line in lines] # Strip newline characters and return the list
            
            path_history = f.read()
            print(f"Successfully read path history {path_history}")
            
    except Exception as e:
        print(f"Error reading file: {e}")
        return path_history
    
    return path_history

def read_file(filename="path_history.txt"):
    
    # DEBUG
    # with open(filename, 'r') as f:
    #     content = f.read()
    #     display.fill(0)
    #     display.text(content, 0, 20)
    #     display.show()

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            # Strip newline characters and return the list
            path_history = [line.strip() for line in lines]
            print(f"path history {path_history}")
    except Exception as e:
        print(f"Error reading file: {e}")
    
    return path_history


def display_show(message: str):
    # Fill the display with black (clear it)
    display.fill(0)

    # Get the length of the message
    message_length = len(message)

    # Define the maximum number of characters per line
    char_per_line = 15

    # Calculate how many lines are needed to display the entire message
    lines_count = (message_length // char_per_line) + (1 if message_length % char_per_line > 0 else 0)

    # Break the message into chunks based on the number of characters per line
    line_message_chunks = [message[i * char_per_line: (i + 1) * char_per_line] for i in range(lines_count)]

    # Display each chunk on the screen, with each line offset by 10 pixels vertically
    for i, line in enumerate(line_message_chunks):
        display.text(line, 0, 10 * i)  # Display each line, offset vertically by 10 * i pixels

    # Refresh the display to show the updated text
    display.show()


def sleep():    
    # delay x milliseconds, usually for transitioning between text on display
    time.sleep_ms(1000)


# DRIVER CODE

initialize()
clear_path_history()

while True:
    solve()

from pololu_3pi_2040_robot import robot
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
current_step = 0  # Global variable to track the current step in the path history
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

    global path_history, current_step
    
    # path_history = read_file()  # Read the path history from the file at startup
    # path_history = ["L", "R", "L", "L", "R", "L", "R"]  # mock for testing
    path_history = read_path_history()
    current_step = 0  # Reset the current step pointer to the beginning of the path history


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


def recall():
    """ Traverses the stored path from beginning of maze """
    print(f"Starting recall navigation")
    
    # global path_history
    
    # Read path history
    # path_history = read_file()
    
    # PROBLEM: problem with real path history read from file because recall() is in the loop "while True", so perhaps store in a global variable and recall() enacted on each direction step instead of entire path history

    # mock_path_history = ["L", "R", "L", "L", "R", "L", "R"]  # mock for testing
    # print(f"path_history {path_history}")
    # print(f"mock path history: {mock_path_history}")

    # Follow each direction in path history
    
    # path_history = path_history[:]  # save copy of path history to global variable to allow other thread to read it safely
    # forward_recall(mock_path_history)  # Move straight until we hit valid junction/intersection, then turn given direction argument
        
        
    # print(f"Completed recall, path history: {path_history}")

    global current_step

    if current_step < len(path_history):
        print(f"Recall step: {current_step}, direction: {path_history[current_step]}")
        direction = path_history[current_step]
        forward_recall(direction)
        
        current_step += 1
    else:
        print("Path completed.")
        display_show("Path completed!")
        end()



def forward_recall(direction):
    
    global p, t1, t2, line, max_speed, run_motors
    
    last_p = 0  
    
    # print(f"forward_recall path history: {path_history}")

    while run_motors:

        # print("---------")
        # print(f"will turn direction {direction}")

        # save a COPY of the line sensor data in a global variable
        # to allow the other thread to read it safely.
        line = line_sensors.read_calibrated()[:]
        # update_display("")
        
        line_sensors.start_read()
        t1 = t2
        t2 = time.ticks_us()
        integral = 0

        # Debug: Print raw sensor readings
        # print(f"Raw sensor readings: {line}")
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
        # display.text(f"PID values -> p: {p}, d: {d}, integral: {integral}", 0, 30)
        # display.show()

        denominator = 10 #decreasing this increases the magnitude of the power_difference - makes turns sharper
        power_difference = (p / denominator + integral / 10000 + d * 3 / 2) 
        
        # Debug: Print power difference
        # print(f"Power difference: {power_difference}")
        # display.text(f"Power difference: {power_difference}", 0, 40)
        # display.show()

        if(power_difference > max_speed):
            power_difference = max_speed
        if(power_difference < -max_speed):
            power_difference = -max_speed

        # Debug: Print final power difference after limiting
        # print(f"Final power difference: {power_difference}")
        # display.text(f"Final Power difference: {power_difference}", 0, 50)
        # display.show()

        if(power_difference < 0): 
            motors.set_speeds(max_speed+power_difference, max_speed)
        else:
            motors.set_speeds(max_speed, max_speed-power_difference)
        

        if is_maze_end():
            # update_display("End")
            display_show("End")
            end()
            break

        # elif (int(line[1]) < 100) and (int(line[2]) < 100) and (int(line[3]) < 100):
        #     # dead end
        #     # print("dead end")
        #     display_show("dead end")

        #     motors.set_speeds(0,0)
            
        #     # direction = "B"

        #     turn_recall("B")


        elif (int(line[0]) > sensor_threshold) or (int(line[4]) > sensor_threshold):
            # possible valid intersection 

            print(f"valid junction: turn: {direction}")
            display_show("valid junction")

            motors.set_speeds(0,0)
            
            keep_moving_straight()    

            # display_show(f"turning {direction}")

            turn_recall(direction)
            
            # gc.collect()

            break


    
def keep_moving_straight():

    # formerly within get_available_directions()

    display.fill(0)
    initial_count = encoders.get_counts()

    try:
        line = line_sensors.read_calibrated()[:]
    except Exception as e:
        return [False, False, False]

    # left_dir = False
    # right_dir = False
    # straight_dir = Falsew

    if int(line[0]) > sensor_threshold:
        # left_dir = True
        encoders
                  
    # if int(line[4]) > sensor_threshold: 
        # right_dir = True
        

    # line up with intersection to check for straight line
    # TODO: refactor into method keep_moving_straight() or keep_moving_forward?
    motors.set_speeds(500,500)
    while not (encoders.get_counts()[0] >= initial_count[0] + int(encorder_count_forward) and encoders.get_counts()[1] >= initial_count[1] + int(encorder_count_forward)):
        # while straight line, maintain moving speed; otherwise, stop
        pass
    motors.set_speeds(0, 0)

    # line = line_sensors.read_calibrated()[:]

    ## This could cause issues, we are trying to evaluate if the line ahead is straight...ish 
    # if (int(line[1]) > sensor_threshold) and (int(line[2]) > sensor_threshold):
    #     straight_dir = True
    # elif (int(line[2]) > sensor_threshold) and (int(line[3]) > sensor_threshold):
    #     straight_dir = True

    # directions = [left_dir, right_dir, straight_dir]

    gc.collect()
    # return directions


def turn_recall(dir: str):

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

def read_path_history(filename="path_history.txt"):
    # Read entire path history list from specified file

    output_path_history = []
    try:
        with open(filename, 'r') as f:
            output_path_history = f.read()
            print(f"Successfully read path history {path_history}")
    except Exception as e:
        print(f"Error reading file: {e}")
    
    return output_path_history
    
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
        path_history = []
    
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

while True:
    recall()

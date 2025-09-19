#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
# AI Classification Competition Element IDs - Push Back
class GameElementsPushBack:
    BLUE_BLOCK = 0
    RED_BLOCK = 1
left_motor_a = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
left_motor_b = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
right_motor_b = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain_inertial = Inertial(Ports.PORT1)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, drivetrain_inertial, 219.44, 320, 40, MM, 1)
Intake = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)
Ramp = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
Outake = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
Outtake_RAISE = DigitalOut(brain.three_wire_port.a)
controller_1 = Controller(PRIMARY)
# AI Vision Color Descriptions
# AI Vision Code Descriptions
ai_vision_2 = AiVision(Ports.PORT2, AiVision.ALL_TAGS, AiVision.ALL_AIOBJS)
TongueMech = DigitalOut(brain.three_wire_port.b)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    drivetrain_inertial.calibrate()
    while drivetrain_inertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


# Calibrate the Drivetrain
calibrate_drivetrain()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      1413D Coding 2025
#	Author:       prafulsaven & snehthakkar & thomassloncen
#	Created:
#	Description:  VEXcode V5 Python Project: Comp
# 
# ------------------------------------------

# imports
from vex import *
import sys

# constant variable
WHEEL_DIAMETER_MM = 82.55
WHEEL_CIRCUM_MM = WHEEL_DIAMETER_MM * math.pi
DEGREES_PER_REV = 360

# tuning values
DRIVE_KP = 0.1
DRIVE_KI = 0
DRIVE_KD = 0

TURN_KP = 0.1
TURN_KI = 0
TURN_KD = 0

# tweaking values
KP_SCALE = 1.8
NONLINEAR_POW = 0.6
INTEGRAL_DECAY = 0.05
DERIV_FILTER = 0.3
STOP_BAND = 5

# global vars: PID
drive_integral = 0.0
drive_prev_error = 0.0
drive_deriv_est = 0.0
turn_integral = 0.0
turn_prev_error = 0.0
turn_deriv_est = 0.0

def reset_pid_state():
    #resets global variables to 0
    global drive_integral, drive_prev_error, drive_deriv_est
    global turn_integral, turn_prev_error, turn_deriv_est
    drive_integral = drive_prev_error = drive_deriv_est = 0.0
    turn_integral = turn_prev_error = turn_deriv_est = 0.0

def compute_pid(error, Kp, Ki, Kd, is_turn=False):
    # PID output computation provided by user
    if is_turn:
        global turn_integral, turn_prev_error, turn_deriv_est
        integral = turn_integral
        prev_error = turn_prev_error
        deriv_est = turn_deriv_est
    else:
        global drive_integral, drive_prev_error, drive_deriv_est
        integral = drive_integral
        prev_error = drive_prev_error
        deriv_est = drive_deriv_est

    # P
    proportional = Kp * error
    
    # I with decay
    integral = (1 - INTEGRAL_DECAY) * integral + Ki * error
    
    # D with a low pass filter
    raw_deriv = error - prev_error
    deriv_est = (1 - DERIV_FILTER) * deriv_est + DERIV_FILTER * raw_deriv
    derivative = Kd * deriv_est
    
    # updates global variables
    if is_turn:
        turn_integral = integral
        turn_prev_error = error
        turn_deriv_est = deriv_est
    else:
        drive_integral = integral
        drive_prev_error = error
        drive_deriv_est = deriv_est
    
    return proportional + integral + derivative

# helper methods
def mm_to_degrees(distance_mm):
    rotations = distance_mm / WHEEL_CIRCUM_MM
    return rotations * DEGREES_PER_REV

def reset_drive_positions():
    left_drive_smart.set_position(0, DEGREES)
    right_drive_smart.set_position(0, DEGREES)

def avg_drive_position():
    return (left_drive_smart.position(DEGREES) + right_drive_smart.position(DEGREES)) / 2

def spin_drive(left_power, right_power):
    left_drive_smart.spin(FORWARD, left_power, PERCENT)
    right_drive_smart.spin(FORWARD, right_power, PERCENT)

def stop_drive():
    left_drive_smart.stop(HOLD)
    right_drive_smart.stop(HOLD)

# Drive PID
def drive_pid_mm(distance_mm, max_power=80):
    reset_pid_state()
    target_degrees = mm_to_degrees(distance_mm)
    reset_drive_positions()
    start_heading = drivetrain_inertial.heading(DEGREES)

    while True:
        # Distance PID
        current_degrees = avg_drive_position()
        drive_error = target_degrees - current_degrees
        
        # non-linear gain change for DPID
        kp_drive = DRIVE_KP * (1 + min(KP_SCALE - 1, abs(drive_error) ** NONLINEAR_POW / 100.0))
        
        base_power = compute_pid(drive_error, kp_drive, DRIVE_KI, DRIVE_KD)
        base_power = max(min(base_power, max_power), -max_power)

        # heading algorithm
        heading_error = start_heading - drivetrain_inertial.heading(DEGREES)
        kp_turn = TURN_KP * (1 + min(KP_SCALE - 1, abs(heading_error) ** NONLINEAR_POW / 100.0))
        correction_power = compute_pid(heading_error, kp_turn, 0, 0, is_turn=True)
        
        # power equation(PID drive direction control)
        spin_drive(- (base_power + correction_power), - (base_power - correction_power))
        
        # exit conditions
        if abs(drive_error) < STOP_BAND and abs(drive_deriv_est) < 0.2:
            break
            
        wait(20, MSEC)
    
    stop_drive()

# Turn PID
def turn_pid(target_angle, max_power=70):
    reset_pid_state()
    drivetrain_inertial.set_heading(0, DEGREES)

    while True:
        current_angle = drivetrain_inertial.heading(DEGREES)
        error = target_angle - current_angle
        # 360 wraparound for heading 
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Non-linear gain change
        kp_turn = TURN_KP * (1 + min(KP_SCALE - 1, abs(error) ** NONLINEAR_POW / 100.0))
        
        # power computation using PID function
        power = compute_pid(error, kp_turn, TURN_KI, TURN_KD, is_turn=True)
        power = max(min(power, max_power), -max_power)

        # power(PID turn direction control)
        spin_drive(-power, power)

        # Exit condition
        if abs(error) < STOP_BAND and abs(turn_deriv_est) < 0.2:
            break

        wait(20, MSEC)
    
    stop_drive()

#### DRIVER CONTROL ######

def drivercontrol():
    while True:
        forward_power = - controller_1.axis3.position()
        turn_power = - controller_1.axis4.position()
        
        # motor power calc
        left_side_power = forward_power + turn_power
        right_side_power = forward_power - turn_power
        
        # deadband for stickdrift
        if abs(left_side_power) < 5 and abs(right_side_power) < 5:
            left_drive_smart.stop()
            right_drive_smart.stop()
        else:
            left_drive_smart.spin(FORWARD, left_side_power, PERCENT)
            right_drive_smart.spin(FORWARD, right_side_power, PERCENT)
        
        #Scoring calc    

        if controller_1.buttonL1.pressing() and controller_1.buttonX.pressing():
            Intake.spin(FORWARD, 100, PERCENT)
            Ramp.spin(REVERSE, 100, PERCENT)
            Outake.stop()  
        elif controller_1.buttonL2.pressing() and controller_1.buttonX.pressing():
            Intake.spin(REVERSE, 100, PERCENT)
            Ramp.spin(FORWARD, 100, PERCENT)
            Outake.stop()  
        elif controller_1.buttonL2.pressing():
            Intake.spin(REVERSE, 100, PERCENT)
            Ramp.spin(FORWARD, 100, PERCENT)
            Outake.spin(REVERSE, 100, PERCENT)
        elif controller_1.buttonL1.pressing():
            Intake.spin(FORWARD, 100, PERCENT)
            Ramp.spin(REVERSE, 100, PERCENT)
            Outake.spin(FORWARD, 100, PERCENT)
        else:
            Intake.stop()
            Ramp.stop()
            Outake.stop()

        if controller_1.buttonR2.pressing():
            Outtake_RAISE.set(True)
        elif controller_1.buttonR1.pressing():
            Outtake_RAISE.set(False)
        else:
            pass

        if controller_1.buttonY.pressing():
            TongueMech.set(True)
        elif controller_1.buttonA.pressing():
            TongueMech.set(False)
        else:
            pass

    wait(5, MSEC)

#### AUTON SELECTION ######

screen_precision = 0
console_precision = 0
controller_1_precision = 0
myVariable = 0

def autonSelection():
    global myVariable
    brain.screen.set_fill_color(Color.RED)
    brain.screen.draw_rectangle(0, 0, 240, 120)
    brain.screen.set_cursor(3, 8)
    brain.screen.print("RED LEFT")
    
    brain.screen.set_fill_color(Color.RED)
    brain.screen.draw_rectangle(240, 0, 240, 120)
    brain.screen.set_cursor(3, 32)
    brain.screen.print("RED RIGHT")
    
    brain.screen.set_fill_color(Color.BLUE)
    brain.screen.draw_rectangle(0, 120, 240, 120)
    brain.screen.set_cursor(10, 8)
    brain.screen.print("BLUE LEFT")
    
    brain.screen.set_fill_color(Color.BLUE)
    brain.screen.draw_rectangle(240, 120, 240, 120)
    brain.screen.set_cursor(10, 32)
    brain.screen.print("BLUE RIGHT")

    while True:
        if brain.screen.pressing():
            x = brain.screen.x_position()
            y = brain.screen.y_position()
            
            if y < 120:
                if x < 240:
                    myVariable = 1 # RED LEFT
                    controller_1.screen.clear_screen()
                    controller_1.screen.set_cursor(1, 1)
                    controller_1.screen.print("RED LEFT selected")
                else:
                    myVariable = 2 # RED RIGHT
                    controller_1.screen.clear_screen()
                    controller_1.screen.set_cursor(1, 1)
                    controller_1.screen.print("RED RIGHT selected")
            elif y > 120:
                if x < 240:
                    myVariable = 3 # BLUE LEFT
                    controller_1.screen.clear_screen()
                    controller_1.screen.set_cursor(1, 1)
                    controller_1.screen.print("BLUE LEFT selected")
                else:
                    myVariable = 4 # BLUE RIGHT
                    controller_1.screen.clear_screen()
                    controller_1.screen.set_cursor(1, 1)
                    controller_1.screen.print("BLUE RIGHT selected")
            
            wait(100, MSEC)
            
        wait(20, MSEC)


def runAuton():
    global myVariable
    if myVariable == 1:
        red_left()
    elif myVariable == 2:
        red_right()
    elif myVariable == 3:
        blue_left()
    elif myVariable == 4:
        blue_right()

def autonomous():
    autonSelection()
    runAuton()


#### AUTON CODE #####

def blue_left():
    try:
        controller_1.screen.print(str("Running Blue Left Auton"))
    except:
        brain.screen.print(str("Controller not connected"))
    
    Intake.spin(FORWARD, 100, PERCENT)
    Ramp.spin(REVERSE, 100, PERCENT)
    drive_pid_mm(985.012)
    turn_pid(-90)
    drive_pid_mm(-500)
    wait(2, SECONDS)
    drive_pid_mm(500)
    turn_pid(180)
    drive_pid_mm(500)
    wait(50, MSEC)
    Intake.spin(FORWARD, 100, PERCENT)
    Ramp.spin(REVERSE, 85, PERCENT)
    Outake.spin(FORWARD, 80, PERCENT)
    wait(2, SECONDS)
    drive_pid_mm(-250)
    turn_pid(-50)
    drive_pid_mm(500)
    wait(5, SECONDS)

def blue_right():
    try:
        controller_1.screen.print(str("Running Blue Right Auton"))
    except:
        brain.screen.print(str("Controller not connected"))
    wait(5, SECONDS)

def red_left():
    try:
        controller_1.screen.print(str("Running Red Left Auton"))
    except:
        brain.screen.print(str("Controller not connected"))
    wait(5, SECONDS)

def red_right():
    try:
        controller_1.screen.print(str("Running Red Right Auton"))
    except:
        brain.screen.print(str("Controller not connected"))
    wait(5, SECONDS)

##### MAIN RUNNING THE PROGRAM ####

def main():
    autonomous()
    drivercontrol()

if __name__ == "__main__":
    main()


# TO DO LIST:
# Tune PID values ( ASAP )
# Make Auton Routines ( IN PROGRESS )
# Color sort? (Auton and Driver Control)
# AI sensor?
# COMP FORMATTING
#
''''
Questions:



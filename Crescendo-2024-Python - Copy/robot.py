import math

import wpilib
import wpimath
import magicbot

import wpimath.controller
import wpimath.filter
import components.chassis.drivetrain
import components.chassis.swervemodule
import components.intake
import components.deflector as deflector



import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions

# # test
from components.chassis.drivetrain import Drivetrain
# from subsystems.swerve_subsystem import SwerveSubsystem   # NEED TO FIX
# # test


class MyRobot(magicbot.MagicRobot):
    drivetrain: components.chassis.drivetrain.Drivetrain
    intake: components.intake.Intake
    deflector: deflector.Deflector

    field_oriented = True

    def createObjects(self) -> None:
        # maincon = self.controller 
        # auxcon = self.controllerAUX
        
        self.controller = wpilib.XboxController(constants.XBOX_USB_PORT)
        self.controllerAUX = wpilib.XboxController(constants.XBOX_USB_PORT_AUX)
        

    def robotPeriodic(self) -> None:
        self.display_on_shuffleboard()
        # self.drivetrain.update_odometry()

    def autonomousInit(self) -> None:
        pass


    def autonomousPeriodic(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        
        # if self.maincon.getBackButtonPressed() or self.auxcon.getBackButtonPressed():
        #     print("Flipping Controllers")
        # self.maincon, self.auxcon = self.auxcon, self.maincon  # Swap the references
        
        # active_con = self.maincon  # Now maincon is the active controller

        
        if self.controller.getBackButtonPressed() or self.controllerAUX.getBackButtonPressed():
            if self.controller.getYButtonPressed():
                self.drivetrain.zero_gyro()

            if self.controller.getRightBumper():
                self.field_oriented = not self.field_oriented

            if self.controller.getRightTriggerAxis() > 0.1:
             self.intake.target = 1
            else: 
                self.intake.target = 0
            
            if self.controller.getXButtonPressed(): 
                self.intake.pivot_motor.set_position(0)
        
            if self.controllerAUX.getAButtonPressed():
                self.deflector.target = 1
        
            if self.controllerAUX.getBButtonPressed():
                self.deflector.target = 0
        elif self.controllerAUX.getBackButtonPressed():
            print()
            
        else:   #Returns code back 
            
            # this is not done yet 
            print
        
        
        
        
        # self.drive_with_joystick(True)

    def drive_with_joystick(self, field_relative: bool) -> None:
        self.drivetrain.x_speed = -utility_functions.filter_input(self.controller.getLeftY()) * constants.MAX_LINEAR_SPEED
        self.drivetrain.y_speed = -utility_functions.filter_input(self.controller.getLeftX()) * constants.MAX_LINEAR_SPEED
        self.drivetrain.rot = -utility_functions.filter_input(self.controller.getRightX()) * constants.MAX_ROTATION_SPEED

        self.drivetrain.field_relative = field_relative
        self.drivetrain.wait_time = self.control_loop_wait_time
    def drive_with_joystickAux(self, field_relative: bool) -> None:
        self.drivetrain.x_speed = -utility_functions.filter_input(self.controllerAUX.getLeftY()) * constants.MAX_LINEAR_SPEED
        self.drivetrain.y_speed = -utility_functions.filter_input(self.controllerAUX.getLeftX()) * constants.MAX_LINEAR_SPEED
        self.drivetrain.rot = -utility_functions.filter_input(self.controllerAUX.getRightX()) * constants.MAX_ROTATION_SPEED

        self.drivetrain.field_relative = field_relative
        self.drivetrain.wait_time = self.control_loop_wait_time

    

    def display_on_shuffleboard(self) -> None:
        # These displays are for callibrating wheel offsets
        '''
        FL_degrees = (360 * self.drivetrain.front_left.turning_encoder.get_position().value) % 360
        wpilib.SmartDashboard.putNumber("FL Degrees", FL_degrees)

        FL_degrees_corrected = FL_degrees - constants.FL_OFFSET
        wpilib.SmartDashboard.putNumber("FL Degrees Corrected", FL_degrees_corrected)

        FR_degrees = (360 * self.drivetrain.front_right.turning_encoder.get_position().value) % 360
        wpilib.SmartDashboard.putNumber("FR Degrees", FR_degrees)

        FR_degrees_corrected = FR_degrees - constants.FR_OFFSET
        wpilib.SmartDashboard.putNumber("FR Degrees Corrected", FR_degrees_corrected)

        BL_degrees = (360 * self.drivetrain.back_left.turning_encoder.get_position().value) % 360
        wpilib.SmartDashboard.putNumber("BL Degrees", BL_degrees)

        BL_degrees_corrected = BL_degrees - constants.BL_OFFSET
        wpilib.SmartDashboard.putNumber("BL Degrees Corrected", BL_degrees_corrected)

        BR_degrees = (360 * self.drivetrain.back_right.turning_encoder.get_position().value) % 360
        wpilib.SmartDashboard.putNumber("BR Degrees", BR_degrees)

        BR_degrees_corrected = BR_degrees - constants.BR_OFFSET
        wpilib.SmartDashboard.putNumber("BR Degrees Corrected", BR_degrees_corrected)
        
        
        # Displays if bot is field or bot oriented. Green is field, and red is bot oriented
        wpilib.SmartDashboard.putBoolean("Field Oriented", self.field_oriented)

        # Displays angle of robot
        wpilib.SmartDashboard.putNumber("Gyro Angle", self.drivetrain.gyro.get_yaw().value % 360)
        '''

        wpilib.SmartDashboard.putNumber("pivot", self.intake.get_position())
        wpilib.SmartDashboard.putBoolean("has note", self.intake.has_note)
        wpilib.SmartDashboard.putNumber("deflector", self.deflector.get_position())
import math
import wpilib


from wpilib import DigitalInput
import wpimath.controller as controller
import wpimath.trajectory as trajectory

from phoenix6 import hardware, configs, signals, controls
from rev import CANSparkMax

import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions

class Intake(): 
    def __init__(self) -> None:
        
        self.intake_rollers_motor = hardware.TalonFX(constants.INTAKE_ROLLERS_CAN, "*")
        self.beam_break = DigitalInput(constants.INTAKE_BEAM_BREAK)
        self.in_shooter = DigitalInput(constants.SHOOTER_BEAM_BREAK)

        self.indexer = CANSparkMax(constants.INDEXER_CAN, CANSparkMax.MotorType.kBrushless)

        self.pivot_configs = configs.TalonFXConfiguration()
        self.pivot_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.pivot_configs.current_limits.supply_current_limit = 2
        self.pivot_configs.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
    
        self.pivot_motor = hardware.TalonFX(constants.INTAKE_PIVOT_CAN, "*")
        self.pivot_motor.configurator.apply(self.pivot_configs)
        
        self.pid_constraints = trajectory.TrapezoidProfile.Constraints(constants.INTAKE_PIVOT_MAX_VELOCITY, 
                                                                              constants.INTAKE_PIVOT_MAX_ACCELERATION)
        self.pivot_pid = controller.ProfiledPIDController(30, 0, 0.5, self.pid_constraints)
        self.pivot_pid.setTolerance(0.03)

        self.pivot_motor.set_position(0)
        self.target = 0
        
        self.has_note = False
        self.intake_timer = wpilib.Timer()
    
    def get_position(self) -> None: 
        return self.pivot_motor.get_position().value/10
    
    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, target_val):
        self._target = target_val

    def at_target_position(self) -> None: 
        return abs(self.target - self.get_position()) <= 0.03

    def execute(self):
        if (self.get_position() <= 0.1) and (self.beam_break.get()): 
            self.has_note = True
        elif self.beam_break.get(): 
            self.intake_timer.start()
            if self.intake_timer.get() > 0.08: 
                self.intake_timer.stop()
                self.intake_timer.reset()
                self.has_note = True
        else: 
            self.has_note = False

        if (not self.at_target_position()) and (self.get_position() >= -0.1) and (self.get_position() <= 1.1):
            pivot_voltage = self.pivot_pid.calculate(self.get_position(), self.target)
        else:
            pivot_voltage = 0

        if (self.get_position() >= 0.97) and not self.has_note:
                roller_voltage = 3
        elif (self.get_position() <= 0.03) and self.has_note:
                roller_voltage = 3
                if self.in_shooter.get(): 
                    self.indexer.set(-0.3)
                else: 
                    self.indexer.set(0)
        else:
            roller_voltage = 0
            if not self.in_shooter.get():
                self.indexer.set(0)


        self.pivot_motor.set_control(controls.VoltageOut(pivot_voltage))
        self.intake_rollers_motor.set_control(controls.VoltageOut(roller_voltage))
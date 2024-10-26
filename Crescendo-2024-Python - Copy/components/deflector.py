import math
import enum

import wpilib
import wpimath.controller as controller
import wpimath.trajectory as trajectory

from phoenix6 import hardware, configs, signals, controls

import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions

class Deflector():
    def __init__(self) -> None:

        self.configs = configs.TalonFXConfiguration()
        self.configs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        self.configs.current_limits.supply_current_limit = 2

        self.deflector_motor = hardware.TalonFX(constants.DEFLECTOR_CAN, "*")
        self.deflector_motor.configurator.apply(self.configs)
        self.deflector_motor.set_position(0)

        self.pid_constraints = trajectory.TrapezoidProfile.Constraints(constants.DEFLECTOR_MAX_VELOCITY, 
                                                                              constants.DEFLECTOR_MAX_ACCELERATION)
        
        self.deflector_pid = controller.ProfiledPIDController(10, 0, 0, self.pid_constraints)
        self.deflector_pid.setTolerance(0.05)
        
        self._target = 0

    def get_position(self):
          return self.deflector_motor.get_position().value/9 
    
    @property
    def target(self):
         return self._target
    
    @target.setter
    def target(self, target_val):
         self._target = target_val

    def at_target_position(self):
         return abs(self.target - self.get_position()) <= 0.05


    def execute(self):
          if(not self.at_target_position()):
              target_voltage = self.deflector_pid.calculate(self.get_position(), self.target)
          else:
               target_voltage = 0
             
          self.deflector_motor.set_control(controls.VoltageOut(target_voltage))
    
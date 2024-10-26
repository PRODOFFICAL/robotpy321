import math

from wpilib import DriverStation
import wpilib
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

import wpimath.interpolation

from phoenix6 import hardware, configs, controls, signals


import utilities.constants as constants
import utilities.ozone_utility_functions as utility_functions

wheel_radius = constants.WHEEL_RADIUS
encoder_resolution = constants.TURN_ENCODER_TICKS
max_rotation_speed = constants.MAX_SINGLE_SWERVE_ROTATION_SPEED
max_rotation_acceleration = constants.MAX_SINGLE_SWERVE_ROTATION_ACCELERATION

class SwerveModule:
    def __init__(
        self,
        drive_motor_id: int,
        turning_motor_id: int,
        turning_encoder_id: int,
        offset: float,
        name: str
        
    ) -> None:
        self._offset = offset
      
        drive_motor_configs = configs.TalonFXConfiguration()
        drive_motor_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        drive_motor_configs.current_limits.stator_current_limit_enable = True
        drive_motor_configs.current_limits.stator_current_limit = 50
        drive_motor_configs.future_proof_configs = True

        self.drive_motor = hardware.talon_fx.TalonFX(drive_motor_id, "*")
        self.drive_motor.clear_sticky_faults()
        self.drive_motor.configurator.apply(drive_motor_configs)

        
        self.turning_motor_configs = configs.TalonFXConfiguration()
        self.turning_motor_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.turning_motor_configs.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        self.turning_motor_configs.future_proof_configs = True

        self.turning_motor = hardware.TalonFX(turning_motor_id, "*")
        self.turning_motor.clear_sticky_faults()
        self.turning_motor.configurator.apply(self.turning_motor_configs)


        self.turning_encoder = hardware.cancoder.CANcoder(turning_encoder_id, "*")

        self.turning_motor.set_control(controls.VoltageOut(0))

        self.name = name

        # Gains are for example purposes only - must be determined for your own robot!
        self.drive_feed_forward = wpimath.controller.SimpleMotorFeedforwardMeters(0.2278, 2.4176, 0)  # How fast motor turns wrt how fast it is commanded to drive

        self.turning_PID_controller = wpimath.controller.ProfiledPIDController(
            constants.TURNING_P,
            constants.TURNING_I,
            constants.TURNING_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                max_rotation_speed,
                max_rotation_acceleration
            )
        )

        self.turning_PID_controller.enableContinuousInput(-math.pi, math.pi)
        self.turning_PID_controller.setTolerance(math.pi/180)



    @property
    def offset(self) -> float:
        return self._offset

    @offset.setter
    def offset(self, offset_value) -> None:
        if isinstance(offset_value, (int, float)):
            self._offset = offset_value
        else:
            raise ValueError("Can't input that value for offset")

    
    def get_encoder_angle(self) -> float:
        sensor_postion = self.turning_encoder.get_position().value
        return sensor_postion * math.tau - math.radians(self.offset)

    def get_motor_velocity(self) -> float:
        self.drive_motor.get_velocity().value * constants.GEAR_RATIO * constants.WHEEL_RADIUS * math.tau

    def get_motor_position(self) -> float:
        self.drive_motor.get_position().value * constants.WHEEL_RADIUS * math.tau
        
    # def get_drive_distance(self) -> float:
    #     rotations = status_signal.StatusSignal.get_latency_compensated_value(self.drive_motor.get_position(), self.drive_motor.get_velocity())
    #     return rotations * math.tau
    
    # def get_turn_distance(self) -> float:
    #     rotations = status_signal.StatusSignal.get_latency_compensated_value(self.turning_motor.get_position(), self.turning_motor.get_velocity())
    #     return rotations * math.tau

    def get_swerve_state(self) -> wpimath.kinematics.SwerveModuleState:
        return wpimath.kinematics.SwerveModuleState(
            self.get_motor_velocity(),
            wpimath.geometry.Rotation2d(self.get_encoder_angle())
        )

    def get_swerve_position(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition(
            10,
            wpimath.geometry.Rotation2d(self.get_encoder_angle())
        )
    

    def set_module_voltage(self, drive_voltage: float, turn_voltage: float) -> None:
        self.drive_motor.set_control(controls.VoltageOut(drive_voltage))
        self.turning_motor.set_control(controls.VoltageOut(turn_voltage))

    def stop_swerve_module(self) -> None:
        self.set_module_voltage(0, 0)
        self.set_desired_state(wpimath.kinematics.SwerveModuleState(
            0, wpimath.geometry.Rotation2d(self.get_encoder_angle())))

    def set_desired_state(
        self, desired_state: wpimath.kinematics.SwerveModuleState
    ) -> None:

        encoder_rotation = wpimath.geometry.Rotation2d(
            self.get_encoder_angle())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desired_state, encoder_rotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoder_rotation).cos()

        drive_output = self.drive_feed_forward.calculate(state.speed)

        turn_output = self.turning_PID_controller.calculate(
            self.get_encoder_angle(),
            state.angle.radians()
        )

        self.set_module_voltage(drive_output, turn_output)
    
from phoenix6 import hardware, configs, signals
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpilib
import math

import utilities.constants as constants
import components.chassis.swervemodule as swervemodule  



class Drivetrain:
    MAX_SPEED = constants.MAX_LINEAR_SPEED
    MAX_ANGULAR_SPEED = constants.MAX_ROTATION_SPEED

    def __init__(self) -> None:
        wheel_base = constants.WHEEL_BASE
        track_width = constants.TRACK_WIDTH

        self.front_left_location = wpimath.geometry.Translation2d(
            wheel_base/2, track_width/2)
        self.front_right_location = wpimath.geometry.Translation2d(
            wheel_base/2, -track_width/2)
        self.back_left_location = wpimath.geometry.Translation2d(
            -wheel_base/2, track_width/2)
        self.back_right_location = wpimath.geometry.Translation2d(
            -wheel_base/2, -track_width/2)

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            offset=constants.FL_OFFSET,
            name="Front Left"
        )

        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            offset=constants.FR_OFFSET,
            name="Front Right"
        )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            offset=constants.BL_OFFSET,
            name="Back Left"
        )

        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            offset=constants.BR_OFFSET,
            name="Back Right"
        )


        self.gyro = hardware.Pigeon2(constants.PIGEON_CAN, "*")

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.get_yaw_value(),
            (
                self.front_left.get_swerve_position(),
                self.front_right.get_swerve_position(),
                self.back_left.get_swerve_position(),
                self.back_right.get_swerve_position()
            )
        )

        # self.zero_gyro()

        self.x_speed: float = 0
        self.y_speed: float = 0
        self.rot: float = 0
        self.field_relative: bool = True
        self.wait_time: float = 0

    def get_yaw_value(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(math.radians(self.gyro.get_yaw().value))

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        period_seconds: float
    ) -> None:


        if field_relative:
            swerve_module_states = self.kinematics.toSwerveModuleStates(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed, y_speed, rot, self.get_yaw_value())
                )
        else:
            swerve_module_states = self.kinematics.toSwerveModuleStates(
                wpimath.kinematics.ChassisSpeeds(x_speed, y_speed, rot))

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states,
            constants.MAX_LINEAR_SPEED
        )

        self.front_left.set_desired_state(swerve_module_states[0])
        self.front_right.set_desired_state(swerve_module_states[1])
        self.back_left.set_desired_state(swerve_module_states[2])
        self.back_right.set_desired_state(swerve_module_states[3])

    def update_odometry(self) -> None:
        self.odometry.update(
            self.get_yaw_value(),
            (
                self.front_left.get_swerve_position(),
                self.front_right.get_swerve_position(),
                self.back_left.get_swerve_position(),
                self.back_right.get_swerve_position()
            )
        )

    def apply_voltage(self, voltage):
        self.back_left.set_module_voltage(voltage, voltage)
        self.back_right.set_module_voltage(voltage, voltage)
        self.front_left.set_module_voltage(voltage, voltage)
        self.front_right.set_module_voltage(voltage, voltage)

    def zero_gyro(self) -> None:
        self.gyro.set_yaw(0)

    def execute(self) -> None:
        self.drive(self.x_speed, self.y_speed, self.rot,
                   self.field_relative, self.wait_time)

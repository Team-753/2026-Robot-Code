import math

import commands2
import wpilib
from phoenix6 import hardware
from wpimath import estimator, geometry, kinematics
import wpimath

import robotConfig as rc
from swerve.swerveModules import SwerveModule


class DriveTrainSubSystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        #Resets Brake Mode After Power Cycle
        self._drive_brake_enabled = False
        wpilib.SmartDashboard.putBoolean("Drive Brake Enabled", False)

        #Define Swerve Modules
        self.front_left = SwerveModule(
            rc.Swerve.frontLeft.driveMotorId,
            rc.Swerve.frontLeft.turningMotorId,
            rc.Swerve.frontLeft.turningEncoderId,
            rc.Swerve.frontLeft.turningEncoderOffset,
        )
        self.front_right = SwerveModule(
            rc.Swerve.frontRight.driveMotorId,
            rc.Swerve.frontRight.turningMotorId,
            rc.Swerve.frontRight.turningEncoderId,
            rc.Swerve.frontRight.turningEncoderOffset,
        )
        self.rear_left = SwerveModule(
            rc.Swerve.rearLeft.driveMotorId,
            rc.Swerve.rearLeft.turningMotorId,
            rc.Swerve.rearLeft.turningEncoderId,
            rc.Swerve.rearLeft.turningEncoderOffset,
        )
        self.rear_right = SwerveModule(
            rc.Swerve.rearRight.driveMotorId,
            rc.Swerve.rearRight.turningMotorId,
            rc.Swerve.rearRight.turningEncoderId,
            rc.Swerve.rearRight.turningEncoderOffset,
        )

        # Pigeon2 gyro (CAN) for heading.
        self.pigeon = hardware.Pigeon2(rc.Swerve.IMU.pigeonId)

        #Geometry + kinematics
        self.track_width = rc.Swerve.Geometry.trackWidthMeters
        self.wheel_base = rc.Swerve.Geometry.wheelBaseMeters
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.track_width / 2.0, self.wheel_base / 2.0),
            geometry.Translation2d(self.track_width / 2.0, -self.wheel_base / 2.0),
            geometry.Translation2d(-self.track_width / 2.0, self.wheel_base / 2.0),
            geometry.Translation2d(-self.track_width / 2.0, -self.wheel_base / 2.0),
        )

        # Pose estimation fuses gyro + module odometry; vision can be added later.
        self.pose_estimator = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_gyro_rotation(),
            self.get_module_positions(),
            geometry.Pose2d(),
            rc.Swerve.PoseEstimation.stateStdDevs,
            rc.Swerve.PoseEstimation.visionStdDevs,
        )

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)



    #Defined Actions
    def set_drive_brake_enabled(self, enabled: bool) -> None:
        if enabled == self._drive_brake_enabled:
            return
        self._drive_brake_enabled = enabled
        self.front_left.set_drive_brake_enabled(enabled)
        self.front_right.set_drive_brake_enabled(enabled)
        self.rear_left.set_drive_brake_enabled(enabled)
        self.rear_right.set_drive_brake_enabled(enabled)
        wpilib.SmartDashboard.putBoolean("Drive Brake Enabled", enabled)
        print(f"Drive brake enabled set to {enabled}.")

    def get_gyro_rotation(self) -> geometry.Rotation2d:
        # Use Pigeon yaw (degrees) and keep the same inversion as prior code.
        yaw_deg = self.pigeon.get_yaw().value
        return geometry.Rotation2d.fromDegrees(360.0 - yaw_deg)

    def get_module_positions(self) -> tuple[kinematics.SwerveModulePosition, ...]: #Check This
        return (
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.rear_left.get_position(),
            self.rear_right.get_position(),
        )

    def get_pose(self) -> geometry.Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: geometry.Pose2d) -> None:
        self.pose_estimator.resetPosition(
            self.get_gyro_rotation(),
            self.get_module_positions(),
            pose,
        )

    def set_swerve_states(
        self,
        x_speed: float,
        y_speed: float,
        omega: float,
        field_relative: bool = True,
    ) -> None:
        # Convert desired chassis motion into module states.
        if field_relative:
            chassis_speeds = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed,
                y_speed,
                omega,
                self.get_pose().rotation(),
            )
        else:
            chassis_speeds = kinematics.ChassisSpeeds(x_speed, y_speed, omega)

        states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        # Limit wheel speeds to the configured max.
        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds( #Might not be needed for this robot
            states,
            rc.Swerve.Speeds.maxLinearSpeedMetersPerSecond,
        )

        self.front_left.set_state(states[0])
        self.front_right.set_state(states[1])
        self.rear_left.set_state(states[2])
        self.rear_right.set_state(states[3])

    def drive_with_joystick(
        self,
        joystick: commands2.button.CommandJoystick,
        field_relative: bool = True,
    ) -> None:
        deadbands = rc.InputDevices.DriverJoystick.Deadbands
        x_input = wpimath.applyDeadband(joystick.getX(), deadbands.xAxis)
        y_input = wpimath.applyDeadband(joystick.getY(), deadbands.yAxis)
        z_input = wpimath.applyDeadband(joystick.getZ(), deadbands.rotationAxis)

        # WPILib joystick Y is typically inverted; negate for forward-positive.
        x_speed = -y_input * rc.Swerve.Speeds.maxLinearSpeedMetersPerSecond
        y_speed = x_input * rc.Swerve.Speeds.maxLinearSpeedMetersPerSecond
        omega = -z_input * rc.Swerve.Speeds.maxAngularSpeedRadiansPerSecond

        self.set_swerve_states(x_speed, y_speed, omega, field_relative)

    def periodic(self) -> None:
        #Telemetry for now; drivetrain motion logic comes later.
        current_pose = self.pose_estimator.update(
            self.get_gyro_rotation(),
            self.get_module_positions(),
        )
        self.field.setRobotPose(current_pose)
        wpilib.SmartDashboard.putNumber("Robot Pose X", current_pose.X())
        wpilib.SmartDashboard.putNumber("Robot Pose Y", current_pose.Y())
        self.front_left.update_temperature()
        self.front_right.update_temperature()
        self.rear_left.update_temperature()
        self.rear_right.update_temperature()

import math

import commands2
import navx
import wpilib
from wpimath import estimator, geometry, kinematics

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

        #NavX gyro (USB) for heading.
        self.navx = navx.AHRS(navx.AHRS.NavXComType.kUSB1, 200)

        #Geometry + kinematics
        self.track_width = rc.Swerve.Geometry.trackWidthMeters
        self.wheel_base = rc.Swerve.Geometry.wheelBaseMeters
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.track_width / 2.0, self.wheel_base / 2.0),
            geometry.Translation2d(self.track_width / 2.0, -self.wheel_base / 2.0),
            geometry.Translation2d(-self.track_width / 2.0, self.wheel_base / 2.0),
            geometry.Translation2d(-self.track_width / 2.0, -self.wheel_base / 2.0),
        )

        #Pose estimation (no vision fused yet, but ready for it).
        self.pose_estimator = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_navx_rotation(),
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

    def get_navx_rotation(self) -> geometry.Rotation2d:
        #Invert if your gyro yaw is reversed relative to module angles.
        return geometry.Rotation2d(math.tau - self.navx.getRotation2d().radians())

    def get_module_positions(self) -> tuple[kinematics.SwerveModulePosition, ...]:
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
            self.get_navx_rotation(),
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
        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states,
            rc.Swerve.Speeds.maxLinearSpeedMetersPerSecond,
        )

        self.front_left.set_state(states[0])
        self.front_right.set_state(states[1])
        self.rear_left.set_state(states[2])
        self.rear_right.set_state(states[3])

    def periodic(self) -> None:
        #Telemetry for now; drivetrain motion logic comes later.
        current_pose = self.pose_estimator.update(
            self.get_navx_rotation(),
            self.get_module_positions(),
        )
        self.field.setRobotPose(current_pose)
        wpilib.SmartDashboard.putNumber("Robot Pose X", current_pose.X())
        wpilib.SmartDashboard.putNumber("Robot Pose Y", current_pose.Y())
        self.front_left.update_temperature()
        self.front_right.update_temperature()
        self.rear_left.update_temperature()
        self.rear_right.update_temperature()

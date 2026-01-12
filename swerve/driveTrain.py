import commands2
import wpilib

import robotConfig as rc
from swerve.swerveModules import SwerveModule


class DriveTrainSubSystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
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

        self._drive_brake_enabled = False
        wpilib.SmartDashboard.putBoolean("Drive Brake Enabled", False)

    def set_drive_brake_enabled(self, enabled: bool) -> None:
        if enabled == self._drive_brake_enabled:
            return
        self._drive_brake_enabled = enabled
        self.front_left.set_drive_brake_enabled(enabled)
        self.front_right.set_drive_brake_enabled(enabled)
        self.rear_left.set_drive_brake_enabled(enabled)
        self.rear_right.set_drive_brake_enabled(enabled)
        wpilib.SmartDashboard.putBoolean("Drive Brake Enabled", enabled)

    def periodic(self) -> None:
        # Telemetry only for now; drivetrain motion logic comes later.
        self.front_left.update_temperature()
        self.front_right.update_temperature()
        self.rear_left.update_temperature()
        self.rear_right.update_temperature()

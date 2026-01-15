import math
import wpilib
from phoenix6 import configs, controls, hardware, signals
from wpimath import geometry, kinematics
import robotConfig as rc


class SwerveModule:

    def __init__(
        self,
        drive_motor_id: int,
        turning_motor_id: int,
        turning_encoder_id: int,
        turning_encoder_offset: float,
    ) -> None:
        self.drive_motor = hardware.TalonFX(drive_motor_id)
        self.turn_motor = hardware.TalonFX(turning_motor_id)
        self.turn_encoder = hardware.CANcoder(turning_encoder_id)
        self._drive_motor_id = drive_motor_id

        self._drive_request = controls.VelocityVoltage(0.0).with_slot(0)
        self._turn_request = controls.PositionVoltage(0.0).with_slot(1)
        self._neutral_request = controls.NeutralOut()
        self._wheel_circumference_m = (
            rc.Swerve.ModuleConstants.wheelDiameterMeters * math.pi
        )
        self._drive_brake_enabled = False

        encoder_config = configs.CANcoderConfiguration()
        encoder_config.magnet_sensor.absolute_sensor_discontinuity_point = 1.0
        encoder_config.magnet_sensor.magnet_offset = turning_encoder_offset
        encoder_config.magnet_sensor.sensor_direction = (
            signals.SensorDirectionValue.CLOCKWISE_POSITIVE
        )
        self.turn_encoder.configurator.apply(encoder_config)

        drive_config = configs.TalonFXConfiguration()
        # Drive motor reports in motor rotations; ratio maps to wheel rotations.
        drive_config.feedback.sensor_to_mechanism_ratio = (
            rc.Swerve.ModuleConstants.driveGearRatio
        )
        drive_config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        self.drive_motor.configurator.apply(drive_config)

        turn_config = configs.TalonFXConfiguration()
        # Use the CANCoder as the absolute turn feedback sensor.
        turn_config.feedback.feedback_remote_sensor_id = turning_encoder_id
        turn_config.feedback.feedback_sensor_source = (
            signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        turn_config.closed_loop_general.continuous_wrap = True
        turn_config.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        self.turn_motor.configurator.apply(turn_config)

        self.turn_motor.set_position(self.turn_encoder.get_absolute_position().value)
        self.turn_motor.set_control(
            self._turn_request.with_position(self.turn_motor.get_position().value)
        )

    def _get_turn_angle(self) -> geometry.Rotation2d:
        return geometry.Rotation2d.fromRotations(
            self.turn_motor.get_position().value
        )

    def get_position(self) -> kinematics.SwerveModulePosition:
        # Distance in meters is wheel rotations * circumference.
        drive_rotations = self.drive_motor.get_position().value
        distance_m = drive_rotations * self._wheel_circumference_m
        return kinematics.SwerveModulePosition(distance_m, self._get_turn_angle())

    def get_state(self) -> kinematics.SwerveModuleState:
        drive_rps = self.drive_motor.get_velocity().value
        speed_mps = drive_rps * self._wheel_circumference_m
        return kinematics.SwerveModuleState(speed_mps, self._get_turn_angle())

    def set_state(self, desired_state: kinematics.SwerveModuleState) -> None:
        optimized = desired_state
        drive_rps = 0.0
        if self._wheel_circumference_m:
            drive_rps = optimized.speed / self._wheel_circumference_m
        turn_rotations = optimized.angle.radians() / math.tau
        self.drive_motor.set_control(self._drive_request.with_velocity(drive_rps))
        self.turn_motor.set_control(self._turn_request.with_position(turn_rotations))

    def set_drive_brake_enabled(self, enabled: bool) -> None:
        if enabled == self._drive_brake_enabled:
            return
        self._drive_brake_enabled = enabled
        motor_output = configs.MotorOutputConfigs()
        motor_output.neutral_mode = (
            signals.NeutralModeValue.BRAKE
            if enabled
            else signals.NeutralModeValue.COAST
        )
        self.drive_motor.configurator.apply(motor_output)

    def update_temperature(self) -> None:
        temp_c = self.drive_motor.get_device_temp().value
        # Publish in Fahrenheit so the drive team sees familiar units.
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        wpilib.SmartDashboard.putNumber(
            f"Swerve/DriveMotor{self._drive_motor_id}/TempF",
            temp_f,
        )

    def stop(self) -> None:
        self.drive_motor.set_control(self._drive_request.with_velocity(0.0))
        self.turn_motor.set_control(
            self._turn_request.with_position(self.turn_motor.get_position().value)
        )

    def set_neutral(self) -> None:
        self.drive_motor.set_control(self._neutral_request)
        self.turn_motor.set_control(self._neutral_request)

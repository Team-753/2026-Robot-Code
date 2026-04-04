from math import pi

import pytest
import wpimath.geometry
import wpimath.kinematics

import Drivetrain.swerveConfig as swerveConfig
from Drivetrain.swerveSubsys import (
    desaturateWheelSpeedsMetersPerSecond,
    wheelSpeedMetersPerSecondToRotationsPerSecond,
)


def test_wheel_speed_conversion_matches_wheel_circumference():
    wheel_circumference = swerveConfig.swerveWheelDiameter * pi

    assert wheelSpeedMetersPerSecondToRotationsPerSecond(wheel_circumference) == pytest.approx(1.0)


def test_desaturation_caps_module_speeds_to_configured_max():
    states = (
        wpimath.kinematics.SwerveModuleState(
            swerveConfig.swerveMaxWheelSpeedMps * 1.5,
            wpimath.geometry.Rotation2d(),
        ),
        wpimath.kinematics.SwerveModuleState(
            swerveConfig.swerveMaxWheelSpeedMps * 1.2,
            wpimath.geometry.Rotation2d(),
        ),
        wpimath.kinematics.SwerveModuleState(
            swerveConfig.swerveMaxWheelSpeedMps * 0.8,
            wpimath.geometry.Rotation2d(),
        ),
        wpimath.kinematics.SwerveModuleState(
            swerveConfig.swerveMaxWheelSpeedMps * 0.5,
            wpimath.geometry.Rotation2d(),
        ),
    )

    desaturated = desaturateWheelSpeedsMetersPerSecond(states)

    assert max(state.speed for state in desaturated) == pytest.approx(swerveConfig.swerveMaxWheelSpeedMps)

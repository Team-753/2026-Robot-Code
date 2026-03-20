import pytest

from AuxilarySystems import auxiliaryConfig
from AuxilarySystems.shooterSubsys import shooterSubsys


def _build_shooter_stub():
    shooter = shooterSubsys.__new__(shooterSubsys)
    shooter.manualTargetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
    shooter.autoTargetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
    shooter.targetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
    shooter.autoVelocityEnabled = True
    shooter.targetDistanceMeters = None
    return shooter


def test_auto_velocity_reference_matches_requested_ratio():
    shooter = _build_shooter_stub()

    velocity = shooter.calculateVelocityForDistance(
        auxiliaryConfig.shooterVelocityReferenceDistanceMeters
    )

    assert velocity == pytest.approx(auxiliaryConfig.shooterVelocityReferenceRps)


def test_manual_velocity_mode_ignores_distance_updates():
    shooter = _build_shooter_stub()
    shooter.manualTargetVelocity = 24.0

    shooter.setAutoVelocityEnabled(False)
    shooter.setTargetDistance(auxiliaryConfig.shooterVelocityReferenceDistanceMeters * 2.0)

    assert shooter.autoTargetVelocity == pytest.approx(
        auxiliaryConfig.shooterVelocityReferenceRps * 2.0
    )
    assert shooter.targetVelocity == pytest.approx(24.0)

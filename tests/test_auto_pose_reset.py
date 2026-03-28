import math

import wpimath.geometry

from Drivetrain.swerveSubsys import driveTrainSubsys


class _FakePoseEstimator:
    def __init__(self):
        self.calls = []

    def resetPosition(self, gyroAngle, wheelPositions, pose):
        self.calls.append((gyroAngle, wheelPositions, pose))


class _FakeField:
    def __init__(self):
        self.pose = None

    def setRobotPose(self, pose):
        self.pose = pose


def test_reset_pose_uses_auto_translation_and_compass_heading():
    fakeEstimator = _FakePoseEstimator()
    fakeField = _FakeField()
    currentYaw = wpimath.geometry.Rotation2d.fromDegrees(37.0)
    wheelPositions = ("modulePositions",)

    fakeDrive = type("FakeDrive", (), {})()
    fakeDrive.poseEstimator = fakeEstimator
    fakeDrive.field = fakeField
    fakeDrive.getRobotYaw = lambda: currentYaw
    fakeDrive.getSwerveState = lambda: wheelPositions

    autoPose = wpimath.geometry.Pose2d(
        2.5,
        4.0,
        wpimath.geometry.Rotation2d.fromDegrees(180.0),
    )

    appliedPose = driveTrainSubsys.resetPose(fakeDrive, autoPose)

    assert len(fakeEstimator.calls) == 1
    gyroAngle, capturedWheelPositions, resetPose = fakeEstimator.calls[0]
    assert gyroAngle == currentYaw
    assert capturedWheelPositions == wheelPositions
    assert math.isclose(resetPose.X(), autoPose.X())
    assert math.isclose(resetPose.Y(), autoPose.Y())
    assert math.isclose(resetPose.rotation().degrees(), currentYaw.degrees())
    assert math.isclose(appliedPose.rotation().degrees(), currentYaw.degrees())
    assert fakeField.pose == appliedPose

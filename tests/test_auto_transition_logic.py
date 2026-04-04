from math import atan2

import commands2
import wpilib
import wpimath.geometry

from AuxilarySystems import auxiliaryConfig
from Drivetrain.Targeting2 import getSpeakerTargetPoint, targetPointCommand
from robotContainer import robotContainer


class _FakeDriveSubsystem:
    def __init__(self, pose):
        self.pose = pose
        self.overidedInputs = [None, None, None]

    def getPoseState(self):
        return self.pose

    def overideInput(self, x=None, y=None, rot=None):
        self.overidedInputs = [x, y, rot]


class _FakeShooterSubsystem:
    def __init__(self):
        self.shoot_starts = 0

    def autoShootStart(self):
        self.shoot_starts += 1


def _build_locked_pose(targetX, targetY, x=2.0, y=2.0):
    desired_rotation = atan2(y - targetY, x - targetX)
    return wpimath.geometry.Pose2d(
        x,
        y,
        wpimath.geometry.Rotation2d(desired_rotation),
    )


def test_transition_waits_for_lock_hold_and_spinup(monkeypatch):
    current_time = [100.0]
    targetX, targetY = getSpeakerTargetPoint()
    container = robotContainer.__new__(robotContainer)
    container.driveSubsystem = _FakeDriveSubsystem(_build_locked_pose(targetX, targetY))
    container.shooterSubsystem = _FakeShooterSubsystem()
    container.autoTransitionTargetPoint = (targetX, targetY)
    container.autoTransitionLockedSince = None
    container.autoTransitionShooterStarted = False
    container.autoTransitionShooterStartTime = None

    monkeypatch.setattr(wpilib.Timer, "getFPGATimestamp", lambda: current_time[0])
    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)

    assert container.shouldStartAutoTransitionIndexer() is False
    assert container.shooterSubsystem.shoot_starts == 0

    current_time[0] += auxiliaryConfig.autoTargetLockHoldSeconds / 2.0
    assert container.shouldStartAutoTransitionIndexer() is False
    assert container.shooterSubsystem.shoot_starts == 0

    current_time[0] += auxiliaryConfig.autoTargetLockHoldSeconds / 2.0 + 0.01
    assert container.shouldStartAutoTransitionIndexer() is False
    assert container.shooterSubsystem.shoot_starts == 1

    current_time[0] += auxiliaryConfig.shooterStartupTime - 0.01
    assert container.shouldStartAutoTransitionIndexer() is False

    current_time[0] += 0.02
    assert container.shouldStartAutoTransitionIndexer() is True


def test_target_point_command_resets_profiled_pid_before_first_execute():
    targetX, targetY = getSpeakerTargetPoint()
    drive = _FakeDriveSubsystem(_build_locked_pose(targetX, targetY))
    command = targetPointCommand(drive, targetX, targetY)

    command.execute()

    assert drive.overidedInputs[2] is not None
    assert abs(drive.overidedInputs[2]) < 1.0


def test_second_auto_only_loads_when_enabled(monkeypatch):
    container = robotContainer.__new__(robotContainer)
    container.getSelectedTrajectoryName = lambda: "Primary"
    container.getSelectedSecondTrajectoryName = lambda: "Second"
    container.isSecondAutoEnabled = lambda: False
    container.createAutoDriveCommand = lambda name: f"drive:{name}"
    container.buildAutoTransitionCommand = lambda: "transition"

    monkeypatch.setattr(
        "robotContainer.commands2.SequentialCommandGroup",
        lambda *commands: list(commands),
    )
    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)

    auto_command = container.buildAutonomousCommand()

    assert auto_command == ["drive:Primary", "transition"]


def test_second_auto_loads_when_enabled(monkeypatch):
    container = robotContainer.__new__(robotContainer)
    container.getSelectedTrajectoryName = lambda: "Primary"
    container.getSelectedSecondTrajectoryName = lambda: "Second"
    container.isSecondAutoEnabled = lambda: True
    container.createAutoDriveCommand = lambda name: f"drive:{name}"
    container.buildAutoTransitionCommand = lambda: "transition"

    monkeypatch.setattr(
        "robotContainer.commands2.SequentialCommandGroup",
        lambda *commands: list(commands),
    )
    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)

    auto_command = container.buildAutonomousCommand()

    assert auto_command == [
        "drive:Primary",
        "transition",
        "drive:Second",
        "transition",
    ]

import wpilib
import wpimath.geometry

from AuxilarySystems.shooterDistanceCommand import setShooterDistanceCommand
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand


class _FakeDrive:
    def getPoseState(self):
        return wpimath.geometry.Pose2d()


class _FakeShooter:
    pass


class _FakeIndexer:
    autoFeedActive = False

    def autoFeedStop(self):
        pass


class _FakeTargetCommand:
    def __init__(self):
        self.executed = 0

    def execute(self):
        self.executed += 1


def test_trigger_distance_command_uses_shared_helper(monkeypatch):
    calls = []
    drive = object()
    shooter = object()
    command = setShooterDistanceCommand(drive, shooter)

    monkeypatch.setattr(
        "AuxilarySystems.shooterDistanceCommand.updateShooterDistanceFromDrivePose",
        lambda drive_subsys, shooter_subsys: calls.append((drive_subsys, shooter_subsys)),
    )

    command.execute()

    assert calls == [(drive, shooter)]


def test_auto_shot_sequence_uses_shared_distance_helper(monkeypatch):
    calls = []
    command = autoDriveTrainCommand.__new__(autoDriveTrainCommand)
    command.driveSubsys = _FakeDrive()
    command.shooterSubsys = _FakeShooter()
    command.indexerSubsys = _FakeIndexer()
    command.autoTargetCommand = _FakeTargetCommand()
    command.autoShotLockedSince = None
    command.autoShotSpinStartTime = None
    command.autoShotShooterStarted = False
    command.autoShotFeedStarted = False

    monkeypatch.setattr(
        "Drivetrain.autonomousDriveSubsys.shooterDistanceCommand.updateShooterDistanceFromDrivePose",
        lambda drive_subsys, shooter_subsys: calls.append((drive_subsys, shooter_subsys)),
    )
    monkeypatch.setattr("Drivetrain.autonomousDriveSubsys.isSpeakerLocked", lambda pose, tolerance: False)
    monkeypatch.setattr(wpilib.Timer, "getFPGATimestamp", lambda: 1.0)

    command.executeAutoShotSequence()

    assert calls == [(command.driveSubsys, command.shooterSubsys)]
    assert command.autoTargetCommand.executed == 1

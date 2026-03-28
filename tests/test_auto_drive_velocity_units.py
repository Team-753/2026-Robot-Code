import wpilib
import wpimath.kinematics

from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand


class _FakeClock:
    def get(self):
        return 0.0


class _FakeTrajectory:
    def sample_at(self, timeSeconds, flipForRedAlliance):
        return object()


class _FakeDriveSubsystem:
    def __init__(self):
        self.calls = []

    def setStateMeters(self, x, y, rot):
        self.calls.append((x, y, rot))

    def setState(self, *args):
        raise AssertionError("auto should use the meter-based drivetrain path")


class _FakeIntakeSubsystem:
    def autoIntakeDown(self):
        pass

    def autoIntakeUp(self):
        pass

    def autoGrabStart(self):
        pass

    def autoGrabStop(self):
        pass


def test_auto_execute_uses_meter_based_drive_command(monkeypatch):
    command = autoDriveTrainCommand.__new__(autoDriveTrainCommand)
    command.traj = _FakeTrajectory()
    command.eventList = []
    command.nextEventIndex = 0
    command.clock = _FakeClock()
    command.trajectoryTotalTime = 1.0
    command.flipForRedAlliance = False
    command.driveSubsys = _FakeDriveSubsystem()
    command.intakeSubsys = _FakeIntakeSubsystem()
    command.shooterState = False
    command.autoTargetCommand = None
    command.intakeDown = True
    command.intakeSpin = False
    command.stopAutoShotSequence = lambda: None
    command.getSpeeds = lambda goal: wpimath.kinematics.ChassisSpeeds(1.2, -0.4, 0.7)

    monkeypatch.setattr(
        wpilib.DriverStation,
        "getAlliance",
        lambda: wpilib.DriverStation.Alliance.kBlue,
    )

    command.execute()

    assert command.driveSubsys.calls == [(1.2, 0.4, 0.7)]

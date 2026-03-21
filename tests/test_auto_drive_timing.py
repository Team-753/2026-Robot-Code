import commands2
import pytest
import wpilib
import wpimath.geometry

from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain import swerveConfig


class _FakeClock:
    def __init__(self, time_seconds):
        self.time_seconds = time_seconds

    def get(self):
        return self.time_seconds

    def reset(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass


class _FakeSample:
    x = 0.0
    y = 0.0
    vx = 0.0
    vy = 0.0
    omega = 0.0
    heading = 0.0


class _FakeTrajectory:
    def __init__(self, total_time):
        self.total_time = total_time
        self.events = []
        self.sample_calls = []

    def get_initial_pose(self, flip_for_red):
        return wpimath.geometry.Pose2d()

    def get_total_time(self):
        return self.total_time

    def sample_at(self, time_seconds, flip_for_red):
        self.sample_calls.append((time_seconds, flip_for_red))
        return _FakeSample()


class _FakeDrive(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.pose = wpimath.geometry.Pose2d()
        self.last_state = None

    def getPoseState(self):
        return self.pose

    def setState(self, fb, lr, rot):
        self.last_state = (fb, lr, rot)

    def overideInput(self, x=None, y=None, rot=None):
        pass


class _FakeShooter:
    def autoInit(self):
        pass

    def autoFeedStop(self):
        pass

    def autoShootStop(self):
        pass

    def isShooting(self):
        return False


class _FakeIntake:
    def autoGrabStop(self):
        pass

    def autoIntakeDown(self):
        pass

    def autoIntakeUp(self):
        pass

    def autoGrabStart(self):
        pass


class _FakeIndexer:
    autoFeedActive = False

    def autoInit(self):
        pass

    def autoFeedStop(self):
        pass

    def autoShootStop(self):
        pass


def test_auto_uses_configured_finish_buffer(monkeypatch):
    fake_trajectory = _FakeTrajectory(total_time=2.0)
    monkeypatch.setattr(wpilib.DriverStation, "getAlliance", lambda: wpilib.DriverStation.Alliance.kBlue)
    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)
    monkeypatch.setattr("Drivetrain.autonomousDriveSubsys.choreo.load_swerve_trajectory", lambda name: fake_trajectory)

    command = autoDriveTrainCommand(_FakeShooter(), _FakeIntake(), _FakeIndexer(), _FakeDrive(), "TestPath")

    assert command.trajectoryTotalTime == pytest.approx(2.0)
    assert command.totalTime == pytest.approx(2.0 + swerveConfig.autoFinishBufferSeconds)


def test_auto_holds_final_sample_during_finish_buffer(monkeypatch):
    fake_trajectory = _FakeTrajectory(total_time=2.0)
    monkeypatch.setattr(wpilib.DriverStation, "getAlliance", lambda: wpilib.DriverStation.Alliance.kBlue)
    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)
    monkeypatch.setattr("Drivetrain.autonomousDriveSubsys.choreo.load_swerve_trajectory", lambda name: fake_trajectory)

    command = autoDriveTrainCommand(_FakeShooter(), _FakeIntake(), _FakeIndexer(), _FakeDrive(), "TestPath")
    command.clock = _FakeClock(2.3)

    command.execute()

    assert fake_trajectory.sample_calls[-1][0] == pytest.approx(2.0)
    assert command.isFinished() is False

    command.clock = _FakeClock(2.0 + swerveConfig.autoFinishBufferSeconds)
    assert command.isFinished() is True

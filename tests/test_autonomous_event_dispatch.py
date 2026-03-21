import wpilib

from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand


def test_auto_event_aliases_dispatch_to_supported_handlers(monkeypatch):
    dispatched = []
    command = autoDriveTrainCommand.__new__(autoDriveTrainCommand)
    command.setShooting = lambda value: dispatched.append(("setShooting", value))
    command.setIntakeSpin = lambda value: dispatched.append(("setIntakeSpin", value))

    monkeypatch.setattr(wpilib.SmartDashboard, "putString", lambda *args, **kwargs: None)

    assert command.runEventExpression("setShooting(True)") is True
    assert command.runEventExpression("setIntakeSping(False)") is True
    assert command.runEventExpression("setIntake(False)") is True

    assert dispatched == [
        ("setShooting", True),
        ("setIntakeSpin", False),
        ("setIntakeSpin", False),
    ]


def test_unknown_auto_event_reports_error(monkeypatch):
    dashboard_strings = {}
    command = autoDriveTrainCommand.__new__(autoDriveTrainCommand)

    def _put_string(key, value):
        dashboard_strings[key] = value

    monkeypatch.setattr(wpilib.SmartDashboard, "putString", _put_string)

    assert command.runEventExpression("setDoesNotExist(True)") is False
    assert "setDoesNotExist(True)" in dashboard_strings["Auto Last Event Error"]

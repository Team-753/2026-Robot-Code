import pytest
import wpilib

from robotContainer import robotContainer


def _build_timer_container():
    container = robotContainer.__new__(robotContainer)
    container.matchModeName = None
    container.matchModeStartTime = None
    return container


def _install_dashboard_capture(monkeypatch):
    dashboard = {}
    monkeypatch.setattr(
        wpilib.SmartDashboard,
        "putString",
        lambda key, value: dashboard.__setitem__(key, value),
    )
    monkeypatch.setattr(
        wpilib.SmartDashboard,
        "putNumber",
        lambda key, value: dashboard.__setitem__(key, float(value)),
    )
    monkeypatch.setattr(
        wpilib.SmartDashboard,
        "putBoolean",
        lambda key, value: dashboard.__setitem__(key, bool(value)),
    )
    return dashboard


def _install_driverstation(monkeypatch, mode, match_time):
    monkeypatch.setattr(wpilib.DriverStation, "isAutonomousEnabled", lambda: mode == "auto")
    monkeypatch.setattr(wpilib.DriverStation, "isTeleopEnabled", lambda: mode == "teleop")
    monkeypatch.setattr(wpilib.DriverStation, "isTestEnabled", lambda: mode == "test")
    monkeypatch.setattr(wpilib.DriverStation, "isDisabled", lambda: mode == "disabled")
    monkeypatch.setattr(wpilib.DriverStation, "getMatchTime", lambda: match_time[0])


def test_match_timer_dashboard_prefers_driverstation_countdown(monkeypatch):
    current_time = [100.0]
    match_time = [20.0]
    container = _build_timer_container()
    dashboard = _install_dashboard_capture(monkeypatch)

    monkeypatch.setattr(wpilib.Timer, "getFPGATimestamp", lambda: current_time[0])
    _install_driverstation(monkeypatch, "auto", match_time)

    container.updateMatchTimerDashboard()

    current_time[0] = 105.0
    match_time[0] = 15.0
    container.updateMatchTimerDashboard()

    assert dashboard["Match Mode"] == "Autonomous"
    assert dashboard["Match Segment"] == "Autonomous"
    assert dashboard["Current Shift"] == "Autonomous"
    assert dashboard["Match Time Source"] == "DriverStation"
    assert dashboard["Match Time Remaining (s)"] == pytest.approx(15.0)
    assert dashboard["Segment Time Remaining (s)"] == pytest.approx(15.0)
    assert dashboard["Current Shift Time Remaining (s)"] == pytest.approx(15.0)
    assert dashboard["DriverStation Match Time (s)"] == pytest.approx(15.0)
    assert dashboard["Endgame Active"] is False


def test_match_timer_dashboard_falls_back_to_local_countdown(monkeypatch):
    current_time = [200.0]
    match_time = [-1.0]
    container = _build_timer_container()
    dashboard = _install_dashboard_capture(monkeypatch)

    monkeypatch.setattr(wpilib.Timer, "getFPGATimestamp", lambda: current_time[0])
    _install_driverstation(monkeypatch, "teleop", match_time)

    container.updateMatchTimerDashboard()

    current_time[0] = 332.0
    container.updateMatchTimerDashboard()

    assert dashboard["Match Mode"] == "Teleop"
    assert dashboard["Match Segment"] == "End Game"
    assert dashboard["Current Shift"] == "End Game"
    assert dashboard["Match Time Source"] == "Local"
    assert dashboard["Match Time Remaining (s)"] == pytest.approx(8.0)
    assert dashboard["Segment Time Remaining (s)"] == pytest.approx(8.0)
    assert dashboard["Current Shift Time Remaining (s)"] == pytest.approx(8.0)
    assert dashboard["DriverStation Match Time (s)"] == pytest.approx(-1.0)
    assert dashboard["Endgame Active"] is True


def test_match_timer_dashboard_reports_2026_shift_segments(monkeypatch):
    current_time = [300.0]
    match_time = [-1.0]
    container = _build_timer_container()
    dashboard = _install_dashboard_capture(monkeypatch)

    monkeypatch.setattr(wpilib.Timer, "getFPGATimestamp", lambda: current_time[0])
    _install_driverstation(monkeypatch, "teleop", match_time)

    container.updateMatchTimerDashboard()

    current_time[0] = 343.0
    container.updateMatchTimerDashboard()

    assert dashboard["Match Mode"] == "Teleop"
    assert dashboard["Match Segment"] == "Shift 2"
    assert dashboard["Current Shift"] == "Shift 2"
    assert dashboard["Match Time Source"] == "Local"
    assert dashboard["Match Time Remaining (s)"] == pytest.approx(97.0)
    assert dashboard["Segment Time Remaining (s)"] == pytest.approx(17.0)
    assert dashboard["Current Shift Time Remaining (s)"] == pytest.approx(17.0)
    assert dashboard["Endgame Active"] is False

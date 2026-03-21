import math

import choreo
import wpilib

from AuxilarySystems import auxiliaryConfig
from Drivetrain.Targeting2 import getSpeakerTargetPoint, targetPointCommand


def _select_auto_path(robot, trajectory_name):
    robot.rContainer.getSelectedTrajectoryName = lambda: trajectory_name
    robot.rContainer.getSelectedSecondTrajectoryName = lambda: ""
    robot.rContainer.isSecondAutoEnabled = lambda: False


def _find_inflight_shooting_path(trajectory_names):
    for trajectory_name in trajectory_names:
        trajectory = choreo.load_swerve_trajectory(trajectory_name)
        total_time = trajectory.get_total_time()
        for event in trajectory.events:
            if event.event == "setShooting(True)" and event.timestamp < (total_time - 0.5):
                return trajectory_name, event.timestamp
    raise AssertionError("No autonomous trajectory keeps shooting active before the path ends")


def test_auto_transition_activates_targeting(control, robot):
    with control.run_robot():
        transition_path = robot.rContainer.trajectoryNames[0]
        _select_auto_path(robot, transition_path)

        control.step_timing(seconds=0.1, autonomous=True, enabled=True)

        primary_auto = robot.rContainer.primaryAutoCommand
        assert primary_auto is not None

        control.step_timing(
            seconds=primary_auto.totalTime + 0.6,
            autonomous=True,
            enabled=True,
        )

        assert robot.rContainer.autoTransitionActive is True
        assert isinstance(robot.rContainer.autoTransitionTargetCommand, targetPointCommand)
        assert robot.rContainer.autoTransitionTargetPoint == getSpeakerTargetPoint()
        assert robot.rContainer.driveSubsystem.overidedInputs[2] is not None
        assert wpilib.SmartDashboard.getBoolean("Auto Transition Active", False) is True
        assert wpilib.SmartDashboard.getString("Auto Transition Status", "") in (
            "Aiming",
            "Spinning Up",
            "Launching",
            "Reacquiring",
        )


def test_auto_shooting_keeps_persistent_targeting_command(control, robot):
    with control.run_robot():
        trajectory_name, shooting_timestamp = _find_inflight_shooting_path(
            robot.rContainer.trajectoryNames
        )
        _select_auto_path(robot, trajectory_name)

        control.step_timing(seconds=0.1, autonomous=True, enabled=True)

        primary_auto = robot.rContainer.primaryAutoCommand
        assert primary_auto is not None

        step_aligned_shoot_time = math.ceil((shooting_timestamp + 0.01) / 0.2) * 0.2
        control.step_timing(
            seconds=max(0.2, step_aligned_shoot_time - 0.2),
            autonomous=True,
            enabled=True,
        )

        assert primary_auto.shooterState is True
        assert primary_auto.autoTargetCommand is not None
        assert isinstance(primary_auto.autoTargetCommand, targetPointCommand)
        assert robot.rContainer.autoTransitionActive is False
        assert robot.rContainer.driveSubsystem.overidedInputs[2] is not None

        auto_target_command = primary_auto.autoTargetCommand

        control.step_timing(seconds=0.2, autonomous=True, enabled=True)

        assert primary_auto.autoTargetCommand is auto_target_command
        assert isinstance(primary_auto.autoTargetCommand, targetPointCommand)


def test_auto_transition_launches_shot(control, robot):
    with control.run_robot():
        transition_path = robot.rContainer.trajectoryNames[0]
        _select_auto_path(robot, transition_path)

        control.step_timing(seconds=0.1, autonomous=True, enabled=True)

        primary_auto = robot.rContainer.primaryAutoCommand
        assert primary_auto is not None

        transition_launch_seconds = (
            primary_auto.totalTime
            + auxiliaryConfig.autoTargetLockHoldSeconds
            + auxiliaryConfig.shooterStartupTime
            + 1.0
        )
        control.step_timing(
            seconds=transition_launch_seconds,
            autonomous=True,
            enabled=True,
        )

        assert robot.rContainer.autoTransitionActive is True
        assert robot.rContainer.autoTransitionShooterStarted is True
        assert robot.rContainer.shooterSubsystem.toggleshoot is True
        assert robot.rContainer.autoTransitionIndexerStarted is True
        assert robot.rContainer.shooterSubsystem.autoFeedEnabled is True
        assert robot.rContainer.indexerSubsystem.autoFeedActive is True
        assert wpilib.SmartDashboard.getString("Auto Transition Status", "") == "Launching"

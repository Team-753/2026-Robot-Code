import math
import wpilib
import wpimath.controller
from typing import Optional, TYPE_CHECKING
import Drivetrain.swerveConfig as swerveConfig

# Target points are in meters, WPILib blue-origin field coordinates.
TARGET_POINT_RED = (4.62507, 4.03514)
TARGET_POINT_BLUE = (11.91497, 4.03514)

# Match the swerve module turn PID values from swerveSubsys.
TURN_KP = 50.0
TURN_KI = 0.0001
TURN_KD = 0.1

if TYPE_CHECKING:
    from Drivetrain.swerveSubsys import driveTrainSubsys



def _clamp(value: float, min_value: float, max_value: float) -> float:
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def _get_alliance():
    try:
        alliance = wpilib.DriverStation.getAlliance()
    except Exception:
        return None

    # Some bindings may return (hasAlliance, alliance)
    if isinstance(alliance, tuple) and len(alliance) == 2 and isinstance(alliance[0], bool):
        return alliance[1] if alliance[0] else None

    # Some bindings may return an Optional-like with .value
    if hasattr(alliance, "value"):
        return alliance.value

    return alliance


def _get_target_point():
    alliance = _get_alliance()
    if alliance == wpilib.DriverStation.Alliance.kRed:
        return TARGET_POINT_RED
    return TARGET_POINT_BLUE


def _pose_xy(pose2d):
    if hasattr(pose2d, "X") and callable(getattr(pose2d, "X")):
        return float(pose2d.X()), float(pose2d.Y())
    return float(pose2d.x), float(pose2d.y)


class Targeting:
    def __init__(self, drive_subsys: Optional["driveTrainSubsys"] = None):
        self.heading_pid = wpimath.controller.PIDController(
            TURN_KP,
            TURN_KI,
            TURN_KD,
        )
        self.drive_subsys = drive_subsys
        self._target_enable = False
        self.heading_pid.enableContinuousInput(-math.pi, math.pi)

    def set_drive_subsys(self, drive_subsys):
        self.drive_subsys = drive_subsys

    def target_Enable(self):
        self._target_enable = True

    def target_Disable(self):
        self._target_enable = False

    def is_enabled(self) -> bool:
        return self._target_enable
    
    def get_override_rotation(self, pose2d=None, compass_degrees: Optional[float] = None) -> float:
        if pose2d is None and self.drive_subsys is not None:
            pose2d = self.drive_subsys.getPoseState()
        if compass_degrees is None and self.drive_subsys is not None:
            compass_degrees = self.drive_subsys.compass.getRotation2d().degrees()

        if pose2d is None:
            return 0.0
        if compass_degrees is None:
            return 0.0

        target_x, target_y = _get_target_point()
        pose_x, pose_y = _pose_xy(pose2d)
        dx = target_x - pose_x
        dy = target_y - pose_y

        target_heading = math.atan2(dy, dx)
        current_heading = math.radians(compass_degrees)
        rot_cmd = self.heading_pid.calculate(current_heading, target_heading)

        return _clamp(rot_cmd, -swerveConfig.driveTurnSpeed, swerveConfig.driveTurnSpeed)

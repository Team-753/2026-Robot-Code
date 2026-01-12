import commands2
import wpilib
import driveTrain 

import robotConfig as rc


class RobotContainer:
    def __init__(self) -> None:

        self.driver_joystick = commands2.button.CommandJoystick(
            rc.InputDevices.DriverJoystick.usbId
        )

        self.driveTrain = DriveTrainSubSystem()

        self.driver_joystick.button(0).onTrue(driveTrain.set_drive_brake_enabled)


        self._brake_enabled = False
        wpilib.SmartDashboard.putBoolean("Drive Brake: ", self._brake_enabled)

        brake_button = self.driver_joystick.button(
            rc.InputDevices.DriverJoystick.Mappings.brake
        )
        brake_button.onTrue(
            commands2.InstantCommand(lambda: self.set_brake_enabled(True))
        )
        brake_button.onFalse(
            commands2.InstantCommand(lambda: self.set_brake_enabled(False))
        )






    def set_brake_enabled(self, enabled: bool) -> None:
        if self._brake_enabled == enabled:
            return
        self._brake_enabled = enabled
        wpilib.SmartDashboard.putBoolean("Drive Brake: ", enabled)

import commands2

import robotConfig as rc
from swerve.driveTrain import DriveTrainSubSystem


class RobotContainer:
    def __init__(self) -> None:

        self.drive_train = DriveTrainSubSystem()

        #Define Joystick 
        self.driver_joystick = commands2.button.CommandJoystick(
            rc.InputDevices.DriverJoystick.usbId
        )

        #Define Aux Controller
        self.aux_controller = commands2.button.CommandXboxController(
            rc.InputDevices.AuxController.usbId
        )

        #Apply Button Bindings
        self.configureButtonBindings()

        # Default drive command uses joystick input with configured deadbands.
        self.drive_train.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.drive_train.drive_with_joystick(self.driver_joystick),
                self.drive_train,
            )
        )

    def configureButtonBindings(self) -> None:

        #Define Brake Button
        brake_button = self.driver_joystick.button(
            rc.InputDevices.DriverJoystick.Mappings.brakeButton
        )

        #While Brake Button is held, enable Brake Mode
        brake_button.whileTrue(
            commands2.StartEndCommand(
                lambda: self.drive_train.set_drive_brake_enabled(True), #When Pressed, Enable
                lambda: self.drive_train.set_drive_brake_enabled(False), #When Released, Disable
                self.drive_train,
            )
        )

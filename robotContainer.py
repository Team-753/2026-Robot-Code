from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand

import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig
class robotContainer():
    def __init__(self):
        if swerveConfig.driveController=="Joystick"or swerveConfig.driveController=="VKBJoystick":
            self.controllerType="Joystick"
        elif swerveConfig.driveController=="XboxController":
            self.controllerType="XboxController"
        exec("self.controller=commands2.button.Command"+str(self.controllerType)+"("+str(swerveConfig.driveControllerSlot)+")")
        #Declare Subystems
        self.driveSubsystem=driveTrainSubsys()
        exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        #self.joystick.setDefaultCommand(testDefCommand(self.joystick))
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem, self.joystick))
        print("containerInited")
        self.buttonBindings()
    def teleopInit(self):
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
    def autoInit(self):
        self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem))
    def buttonBindings(self):
        print("bindings configed")
        hid = self.controller.getHID()
        print(f"HID buttons: {hid.getButtonCount()} targetingButton: {swerveConfig.targetingButton}")

        def _enable_targeting():
            print("Targeting enabled")
            self.driveSubsystem.setTargetingActive(True)

        def _disable_targeting():
            print("Targeting disabled")
            self.driveSubsystem.setTargetingActive(False)

        self.controller.button(swerveConfig.targetingButton).onTrue(
            commands2.InstantCommand(_enable_targeting)
        )
        self.controller.button(swerveConfig.targetingButton).onFalse(
            commands2.InstantCommand(_disable_targeting)
        )

from swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,autoDriveTrainCommand
import wpilib,commands2,swerveConfig
class robotContainer():
    def __init__(self):
        exec("self.controller=commands2.button.Command"+str(swerveConfig.driveController)+"("+str(swerveConfig.driveControllerSlot)+")")
        #Declare Subystems
        self.driveSubsystem=driveTrainSubsys()
        exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        #self.joystick.setDefaultCommand(testDefCommand(self.joystick))
        exec("self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,joySubsys="+str(swerveConfig.driveController)+"Subsys))")
        print("containerInited")
        self.buttonBindings()
    def teleopInit(self):
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
    def autoInit(self):
        self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem))
    def buttonBindings(self):
        print("bindings configed")
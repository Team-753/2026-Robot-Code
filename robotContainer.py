from swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,autoDriveTrainCommand,VKBJoystickSubsys
import wpilib,commands2,swerveConfig
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

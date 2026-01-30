from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput
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
        if swerveConfig.driveController=="Joystick":
            self.controller.trigger().whileTrue(fieldOrientReorient(self.driveSubsystem))
        if swerveConfig.driveController=="VKBJoystick":
            self.controller.button(0).whileTrue(fieldOrientReorient(self.driveSubsystem))
        if swerveConfig.driveController=="XboxControler":
            self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))
        #self.controller.button(2).whileTrue(overideRobotInput(self.driveSubsystem,x=0.1))
        print("bindings configed")

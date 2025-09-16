from swerveSubsys import driveTrainCommand,joystickSubsys,driveTrainSubsys,hotasSubsys,autoDriveTrainCommand
import wpilib,commands2
class robotContainer():
    def __init__(self):
        self.controller=commands2.button.CommandXboxController(0)
        #Declare Subystems
        self.driveSubsystem=driveTrainSubsys()
        self.joystick=joystickSubsys(self.controller)
        #self.joystick.setDefaultCommand(testDefCommand(self.joystick))
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        print("containerInited")
        self.buttonBindings()
    def teleopInit(self):
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
    def autoInit(self):
        self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem))
    def buttonBindings(self):
        print("bindings configed")
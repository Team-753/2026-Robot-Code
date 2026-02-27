import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig

##IMPORT FROM Drivetrain
from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain.Targeting2 import targetPointCommand

##IMPORT FROM AuxiliarySystems
from AuxilarySystems import auxiliaryConfig, shooterSubsys, IndexerSubsys, IntakeSubsys, flipSubsys


class robotContainer():
    def __init__(self):
        # if swerveConfig.driveController=="Joystick"or swerveConfig.driveController=="VKBJoystick":
        #     self.controllerType="Joystick"
        # elif swerveConfig.driveController=="XboxController":
        #     self.controllerType="XboxController"
        # exec("self.controller=commands2.button.Command"+str(self.controllerType)+"("+str(swerveConfig.driveControllerSlot)+")")
        # exec("self.auxController=commands2.button.Command"+str(auxiliaryConfig.auxController)+"("+str(auxiliaryConfig.auxControllerSlot)+")")
        #Declare Subystems
        # self.driveSubsystem=driveTrainSubsys()
        #self.shooterSubsystem=shooterSubsys.shooterSubsys()
        #self.indexerSubsystem=IndexerSubsys.indexerSubsys()
        #self.intakeSubsystem=IntakeSubsys.intakeSubsys()
        self.flipSubsystem=flipSubsys.flipsubsys()
        # exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        
        # #Set default Command (runs over and over)
        # self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        # print("containerInited")

        # #Set all the binding in the button bindings function
        # self.buttonBindings()
    def teleopInit(self):
        # self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
         #self.shooterSubsystem.teleopInit()
         #self.indexerSubsystem.teleopInit()
         #self.intakeSubsystem.teleopInit()
         self.flipSubsystem.teleopInit()
    def autoInit(self):
    #     self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem))
    # def buttonBindings(self):

    #     ##Stick recenter bindings
    #     if swerveConfig.driveController=="Joystick":
    #         self.controller.trigger().whileTrue(fieldOrientReorient(self.driveSubsystem))
    #     if swerveConfig.driveController=="VKBJoystick":
    #         self.controller.button(1).whileTrue(fieldOrientReorient(self.driveSubsystem))
    #     if swerveConfig.driveController=="XboxController":
    #         self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))

    #     if swerveConfig.driveController=="Joystick":
    #         self.controller.button(2).whileTrue()

    #     ##Shooter bindings
    #     if auxiliaryConfig.auxController=="XboxController":
    #         #self.auxController.a().whileTrue(commands2.RepeatCommand(shooterSubsys.shootBalls(self.shooterSubsystem,0.3,2000)))
    #         self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,1,1)))
    #         self.auxController.y().whileTrue(commands2.RepeatCommand(overideRobotInput(self.driveSubsystem,theta=0.1)))
    #     #self.controller.button(2).whileTrue(overideRobotInput(self.driveSubsystem,theta=0))
    #     print("bindings configed")
    #     hid = self.controller.getHID()
    #     print(f"HID buttons: {hid.getButtonCount()} targetingButton: {swerveConfig.targetingButton}")

    #     def _enable_targeting():
    #         print("Targeting enabled")
    #         self.driveSubsystem.setTargetingActive(True)

    #     def _disable_targeting():
    #         print("Targeting disabled")
    #         self.driveSubsystem.setTargetingActive(False)

    #     self.controller.button(swerveConfig.targetingButton).onTrue(
    #         commands2.InstantCommand(_enable_targeting)
    #     )
    #     self.controller.button(swerveConfig.targetingButton).onFalse(
    #         commands2.InstantCommand(_disable_targeting)
    #     )
            pass
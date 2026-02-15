import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig
from AuxilarySystems.cannonSubsys import * 
import AuxilarySystems.cannonSubsys
import AuxilarySystems.elevatorSubsys
from AuxilarySystems.elevatorSubsys import elevatorUp,elevatorDown,elevatorToPos,elevatorSubSystem
##IMPORT FROM Drivetrain
from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain.Targeting2 import targetPointCommand
##IMPORT FROM AuxiliarySystems
from AuxilarySystems import auxiliaryConfig

class robotContainer():
    def __init__(self):
        if swerveConfig.driveController=="Joystick"or swerveConfig.driveController=="VKBJoystick":
            self.controllerType="Joystick"
        elif swerveConfig.driveController=="XboxController":
            self.controllerType="XboxController"
        exec("self.controller=commands2.button.Command"+str(self.controllerType)+"("+str(swerveConfig.driveControllerSlot)+")")
        exec("self.auxController=commands2.button.Command"+str(auxiliaryConfig.auxController)+"("+str(auxiliaryConfig.auxControllerSlot)+")")
        #Declare Subystems
        self.driveSubsystem=driveTrainSubsys()
        self.elevator=elevatorSubSystem()
        self.cannon=CannonSubsystem()
        exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        
        #Set default Command (runs over and over)
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        print("containerInited")

        #Set all the binding in the button bindings function
        self.buttonBindings()
    def teleopInit(self):
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
    def autoInit(self):
        self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem))
    def buttonBindings(self):

        ##Stick recenter bindings
        if swerveConfig.driveController=="Joystick":
            self.controller.trigger().whileTrue(fieldOrientReorient(self.driveSubsystem))
        if swerveConfig.driveController=="VKBJoystick":
            self.controller.button(1).whileTrue(fieldOrientReorient(self.driveSubsystem))
        if swerveConfig.driveController=="XboxController":
            self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))

        if swerveConfig.driveController=="Joystick":
            self.controller.button(2).whileTrue(targetPointCommand(self.driveSubsystem,4.62507, 4.03514))
        if swerveConfig.driveController=="XboxController":
            self.controller.x().whileTrue(targetPointCommand(self.driveSubsystem,1,1))

        ##Shooter bindings
        if auxiliaryConfig.auxController=="XboxController":
            self.auxController.rightTrigger(0.5).whileTrue(place(self.cannon)) 
            self.auxController.leftTrigger(0.5).whileTrue(intake(self.cannon))

            
            
            self.auxController.a().onTrue(elevatorToPos(self.elevator,6))
            self.auxController.b().onTrue(elevatorToPos(self.elevator,13))
            self.auxController.y().onTrue(elevatorToPos(self.elevator,26))
            self.auxController.x().onTrue(elevatorToPos(self.elevator,0))
            #6/1

            self.auxController.a().onTrue(cannonToPosition(self.cannon, 0.108))
            self.auxController.b().onTrue(cannonToPosition(self.cannon, 0.133))
            self.auxController.y().onTrue(cannonToPosition(self.cannon, 0.175))
            self.auxController.x().onTrue(cannonToPosition(self.cannon, 0.31))

            self.auxController.axisGreaterThan(1,.5).whileTrue(elevatorDown(self.elevator))
            self.auxController.axisLessThan(1,-.5).whileTrue(elevatorUp(self.elevator))
            
            self.auxController.axisLessThan(5,-.5).whileTrue(PivotUp(self.cannon))
            self.auxController.axisGreaterThan(5,.5).whileTrue(PivotDown(self.cannon))
        
        #self.controller.button(2).whileTrue(overideRobotInput(self.driveSubsystem,theta=0))
        print("bindings configed")


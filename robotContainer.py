import os
import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig

##IMPORT FROM Drivetrain
from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput,pointToVelocityVectorCommand
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain.Targeting2 import targetPointCommand

##IMPORT FROM AuxiliarySystems
from AuxilarySystems import auxiliaryConfig, shooterSubsys, IndexerSubsys, IntakeSubsys, flipSubsys

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
        
        self.shooterSubsystem=shooterSubsys.shooterSubsys()
        self.indexerSubsystem=IndexerSubsys.indexerSubsys()
        self.intakeSubsystem=IntakeSubsys.intakeSubsys()
        #self.flipSubsystem=flipSubsys.flipsubsys()

        exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        self.initializeTrajectoryChooser()
        
        #Set default Command (runs over and over)
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        print("containerInited")

        #Set all the binding in the button bindings function
        self.buttonBindings()

    def getTrajectoryNames(self):
        choreoDir = os.path.join(wpilib.getDeployDirectory(),"choreo")
        if not os.path.isdir(choreoDir):
            return []

        names=[]
        for fileName in os.listdir(choreoDir):
            filePath=os.path.join(choreoDir,fileName)
            if os.path.isfile(filePath) and fileName.lower().endswith(".traj"):
                names.append(os.path.splitext(fileName)[0])
        return sorted(names)

    def initializeTrajectoryChooser(self):
        self.trajectoryChooser=wpilib.SendableChooser()
        self.trajectoryNames=self.getTrajectoryNames()

        if self.trajectoryNames:
            self.trajectoryChooser.setDefaultOption(self.trajectoryNames[0],self.trajectoryNames[0])
            for name in self.trajectoryNames[1:]:
                self.trajectoryChooser.addOption(name,name)
        else:
            self.trajectoryChooser.setDefaultOption("No .traj files found","")

        wpilib.SmartDashboard.putData("Auto Trajectory",self.trajectoryChooser)
        wpilib.SmartDashboard.putStringArray("Auto Trajectory Names",self.trajectoryNames)

    def getSelectedTrajectoryName(self):
        selected=self.trajectoryChooser.getSelected()
        if selected is None:
            if self.trajectoryNames:
                selected=self.trajectoryNames[0]
            else:
                selected=""
        wpilib.SmartDashboard.putString("Auto Trajectory Selected",selected if selected else "None")
        return selected

    def teleopInit(self):
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        self.shooterSubsystem.teleopInit()
        self.indexerSubsystem.teleopInit()
        self.intakeSubsystem.teleopInit()
        #self.flipSubsystem.teleopInit()

    def autoInit(self):
        selectedTrajectoryName=self.getSelectedTrajectoryName()
        self.driveSubsystem.setDefaultCommand(autoDriveTrainCommand(self.driveSubsystem,selectedTrajectoryName))
    def buttonBindings(self):

        ##Stick recenter bindings
        if swerveConfig.driveController=="Joystick":
            self.controller.button(6).whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.button(2).whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,11.91497, 4.03514)))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if swerveConfig.driveController=="VKBJoystick":
            self.controller.button(15).whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.button(3).whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,11.91497, 4.03514)))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if swerveConfig.driveController=="XboxController":
            self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,11.91497, 4.03514)))
            self.controller.rightTrigger().whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))

        ##Shooter bindings
        if auxiliaryConfig.auxController=="XboxController":
            #self.auxController.a().whileTrue(commands2.RepeatCommand(shooterSubsys.shootBalls(self.shooterSubsystem,0.3,2000)))
            #self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
            #self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
            #self.auxController.y().whileTrue(commands2.RepeatCommand(overideRobotInput(self.driveSubsystem,theta=0.1)))
            pass
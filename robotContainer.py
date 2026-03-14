import os
import choreo
import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig
# disable warnings about the joystick
wpilib.DriverStation.silenceJoystickConnectionWarning(True)

#IMPORT FROM Drivetrain
from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput,pointToVelocityVectorCommand
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain.Targeting2 import targetPointCommand,targetPointWithLeadCommand

##IMPORT FROM AuxiliarySystems
from AuxilarySystems import auxiliaryConfig, shooterSubsys, IndexerSubsys, IntakeSubsys, flipSubsys

class robotContainer():
    def __init__(self):
        self.autoCommand=None
        self.primaryAutoCommand=None
        self.previewedTrajectoryName=None
        self.previewedFlipForRedAlliance=None
        self.secondAutoTransitionDelaySeconds=0.5

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
        self.flipSubsystem=flipSubsys.flipsubsys()

        exec("self.joystick="+str(swerveConfig.driveController)+"Subsys(self.controller)")
        self.initializeTrajectoryChooser()
        self.updateTrajectoryPreview(force=True)
        
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
        self.secondTrajectoryChooser=wpilib.SendableChooser()
        self.trajectoryNames=self.getTrajectoryNames()

        self.configureTrajectoryChooser(self.trajectoryChooser)
        self.configureTrajectoryChooser(self.secondTrajectoryChooser)

        wpilib.SmartDashboard.putData("Auto Trajectory",self.trajectoryChooser)
        wpilib.SmartDashboard.putData("Second Auto Trajectory",self.secondTrajectoryChooser)
        wpilib.SmartDashboard.putStringArray("Auto Trajectory Names",self.trajectoryNames)
        wpilib.SmartDashboard.putBoolean("Enable Second Auto",False)
        wpilib.SmartDashboard.putString("Second Auto Trajectory Selected","None")
        wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")

    def configureTrajectoryChooser(self,chooser):
        if self.trajectoryNames:
            chooser.setDefaultOption(self.trajectoryNames[0],self.trajectoryNames[0])
            for name in self.trajectoryNames[1:]:
                chooser.addOption(name,name)
        else:
            chooser.setDefaultOption("No .traj files found","")

    def getChooserSelection(self,chooser):
        selected=chooser.getSelected()
        if selected is None:
            if self.trajectoryNames:
                return self.trajectoryNames[0]
            return ""
        return selected

    def getSelectedTrajectoryName(self):
        selected=self.getChooserSelection(self.trajectoryChooser)
        wpilib.SmartDashboard.putString("Auto Trajectory Selected",selected if selected else "None")
        return selected

    def getSelectedSecondTrajectoryName(self):
        selected=self.getChooserSelection(self.secondTrajectoryChooser)
        wpilib.SmartDashboard.putString("Second Auto Trajectory Selected",selected if selected else "None")
        return selected

    def isSecondAutoEnabled(self):
        return wpilib.SmartDashboard.getBoolean("Enable Second Auto",False)

    def shouldFlipTrajectoryForAlliance(self):
        alliance=wpilib.DriverStation.getAlliance()
        return alliance == wpilib.DriverStation.Alliance.kRed

    def getTrajectoryAllianceLabel(self):
        return "Red" if self.shouldFlipTrajectoryForAlliance() else "Blue"

    def updateTrajectoryPreview(self,force=False):
        selectedTrajectoryName=self.getSelectedTrajectoryName()
        self.getSelectedSecondTrajectoryName()
        flipForRedAlliance=self.shouldFlipTrajectoryForAlliance()
        if not force and selectedTrajectoryName==self.previewedTrajectoryName and flipForRedAlliance==self.previewedFlipForRedAlliance:
            return

        self.previewedTrajectoryName=selectedTrajectoryName
        self.previewedFlipForRedAlliance=flipForRedAlliance
        previewPathObject=self.driveSubsystem.field.getObject("Auto Preview Path")
        previewStartObject=self.driveSubsystem.field.getObject("Auto Preview Start")
        previewEndObject=self.driveSubsystem.field.getObject("Auto Preview End")
        wpilib.SmartDashboard.putString("Auto Preview Alliance",self.getTrajectoryAllianceLabel())

        if not selectedTrajectoryName:
            previewPathObject.setPoses([])
            previewStartObject.setPoses([])
            previewEndObject.setPoses([])
            wpilib.SmartDashboard.putString("Auto Preview Trajectory","None")
            wpilib.SmartDashboard.putString("Auto Preview Start Pose","Unavailable")
            wpilib.SmartDashboard.putString("Auto Preview End Pose","Unavailable")
            wpilib.SmartDashboard.putString("Auto Preview Error","No trajectory selected")
            return

        try:
            trajectory=choreo.load_swerve_trajectory(selectedTrajectoryName)
            pathPoses=[sample.flipped().get_pose() if flipForRedAlliance else sample.get_pose() for sample in trajectory.samples]
            initialPose=trajectory.get_initial_pose(flipForRedAlliance)
            finalPose=trajectory.get_final_pose(flipForRedAlliance)

            previewPathObject.setPoses(pathPoses)
            if initialPose is not None:
                previewStartObject.setPose(initialPose)
                wpilib.SmartDashboard.putString("Auto Preview Start Pose",f"{initialPose.X():.3f}, {initialPose.Y():.3f}, {initialPose.rotation().degrees():.1f}")
            else:
                previewStartObject.setPoses([])
                wpilib.SmartDashboard.putString("Auto Preview Start Pose","Unavailable")

            if finalPose is not None:
                previewEndObject.setPose(finalPose)
                wpilib.SmartDashboard.putString("Auto Preview End Pose",f"{finalPose.X():.3f}, {finalPose.Y():.3f}, {finalPose.rotation().degrees():.1f}")
            else:
                previewEndObject.setPoses([])
                wpilib.SmartDashboard.putString("Auto Preview End Pose","Unavailable")

            wpilib.SmartDashboard.putString("Auto Preview Trajectory",selectedTrajectoryName)
            wpilib.SmartDashboard.putString("Auto Preview Error","")
        except Exception as ex:
            previewPathObject.setPoses([])
            previewStartObject.setPoses([])
            previewEndObject.setPoses([])
            wpilib.SmartDashboard.putString("Auto Preview Trajectory",selectedTrajectoryName)
            wpilib.SmartDashboard.putString("Auto Preview Start Pose","Unavailable")
            wpilib.SmartDashboard.putString("Auto Preview End Pose","Unavailable")
            wpilib.SmartDashboard.putString("Auto Preview Error",str(ex))

    def teleopInit(self):
        self.cancelAutonomousCommand()
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        self.shooterSubsystem.teleopInit()
        self.indexerSubsystem.teleopInit()
        self.intakeSubsystem.teleopInit()
        wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")
        #self.flipSubsystem.teleopInit()
        print('entering teleop')

    def autoInit(self):
        self.cancelAutonomousCommand()
        self.autoCommand=self.buildAutonomousCommand()
        initialPose=self.primaryAutoCommand.getInitialPose()
        # Align the estimator with the chosen path so the first sample is field-correct.
        if initialPose is not None:
            self.driveSubsystem.resetPose(initialPose)
            wpilib.SmartDashboard.putString("Auto Start Pose",f"{initialPose.X():.3f}, {initialPose.Y():.3f}, {initialPose.rotation().degrees():.1f}")
        else:
            wpilib.SmartDashboard.putString("Auto Start Pose","Unavailable")
        self.shooterSubsystem.autoInit()
        self.intakeSubsystem.autoInit()
        self.indexerSubsystem.autoInit()
        commands2.CommandScheduler.getInstance().schedule(self.autoCommand)
        #self.flipSubsystem.autoInit()
        print('entering auto')
    
    def autoPeriodic(self):
        pass

    def disabledInit(self): # keep states in subsystems clean by entering disabled mode
        self.cancelAutonomousCommand()
        self.shooterSubsystem.setToIdle()
        self.intakeSubsystem.setToIdle()
        self.indexerSubsystem.setToIdle()
        self.flipSubsystem.setToIdle()
        wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")
        print('entering disabled')

    def cancelAutonomousCommand(self):
        if self.autoCommand is not None and self.autoCommand.isScheduled():
            self.autoCommand.cancel()

    def createAutoDriveCommand(self,trajectoryName):
        return autoDriveTrainCommand(self.shooterSubsystem,self.intakeSubsystem,self.indexerSubsystem,self.driveSubsystem,trajectoryName)

    def beginSecondAutoTransition(self):
        self.driveSubsystem.setState(0,0,0)
        self.shooterSubsystem.autoShootStop()
        self.indexerSubsystem.autoShootStop()
        self.intakeSubsystem.autoGrabStop()
        wpilib.SmartDashboard.putString("Auto Transition Status","Running")

    def finishSecondAutoTransition(self):
        self.shooterSubsystem.autoInit()
        self.intakeSubsystem.autoInit()
        self.indexerSubsystem.autoInit()
        wpilib.SmartDashboard.putString("Auto Transition Status","Complete")

    def buildSecondAutoTransitionCommand(self):
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.beginSecondAutoTransition,self.driveSubsystem),
            commands2.WaitCommand(self.secondAutoTransitionDelaySeconds),
            commands2.InstantCommand(self.finishSecondAutoTransition,self.driveSubsystem),
        )

    def buildAutonomousCommand(self):
        selectedTrajectoryName=self.getSelectedTrajectoryName()
        selectedSecondTrajectoryName=self.getSelectedSecondTrajectoryName()
        self.primaryAutoCommand=self.createAutoDriveCommand(selectedTrajectoryName)
        autoCommands=[self.primaryAutoCommand]

        if self.isSecondAutoEnabled() and selectedSecondTrajectoryName:
            autoCommands.append(self.buildSecondAutoTransitionCommand())
            autoCommands.append(self.createAutoDriveCommand(selectedSecondTrajectoryName))
            wpilib.SmartDashboard.putString("Auto Transition Status","Ready")
        else:
            wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")

        if len(autoCommands)==1:
            return autoCommands[0]
        return commands2.SequentialCommandGroup(*autoCommands)

    def buttonBindings(self):

        ##Stick recenter bindings
        if swerveConfig.driveController=="Joystick":
            self.controller.button(6).whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.button(2).whileTrue(commands2.RepeatCommand(targetPointWithLeadCommand(self.driveSubsystem)))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if swerveConfig.driveController=="VKBJoystick":
            self.controller.button(15).whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.button(3).whileTrue(commands2.RepeatCommand(targetPointWithLeadCommand(self.driveSubsystem)))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if swerveConfig.driveController=="XboxController":
            self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,11.91497, 4.03514)))
            self.controller.rightTrigger().whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if True:
            self.auxController.povUp().whileTrue(flipSubsys.flipGrabberCommand(self.flipSubsystem,-0.5))
            self.auxController.povDown().whileTrue(flipSubsys.flipGrabberCommand(self.flipSubsystem,0.1))
            self.auxController.povLeft().whileTrue(flipSubsys.flipTrackCommand(self.flipSubsystem,0.5))
            self.auxController.povRight().whileTrue(flipSubsys.flipTrackCommand(self.flipSubsystem,-0.5))

            pass

        ##Shooter bindings
        #if auxiliaryConfig.auxController=="XboxController":
            #self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
           # self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
            #self.auxController.y().whileTrue(commands2.RepeatCommand(overideRobotInput(self.driveSubsystem,theta=0.1)))
            #pass
    def testInit(self):
        #self.flipSubsystem.testInit()
        pass

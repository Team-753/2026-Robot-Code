import os
from math import atan2, pi
import choreo
import wpilib,commands2,Drivetrain.swerveConfig as swerveConfig
# disable warnings about the joystick
wpilib.DriverStation.silenceJoystickConnectionWarning(True)

AUTO_PERIOD_SECONDS = 20.0
TELEOP_PERIOD_SECONDS = 140.0
TRANSITION_SHIFT_SECONDS = 10.0
ALLIANCE_SHIFT_SECONDS = 25.0
ENDGAME_PERIOD_SECONDS = 30.0

#IMPORT FROM Drivetrain
from Drivetrain.swerveSubsys import driveTrainCommand,JoystickSubsys,driveTrainSubsys,XboxControllerSubsys,VKBJoystickSubsys,fieldOrientReorient,overideRobotInput,pointToVelocityVectorCommand,setSwerveXMode
from Drivetrain.autonomousDriveSubsys import autoDriveTrainCommand
from Drivetrain.Targeting2 import getSpeakerDistanceMeters,getSpeakerTargetPoint,getTargetAimErrorDegrees,isTargetLocked,targetPointCommand

##IMPORT FROM AuxiliarySystems
from AuxilarySystems import auxiliaryConfig, shooterSubsys, IndexerSubsys, IntakeSubsys, flipSubsys, shooterDistanceCommand

class robotContainer():
    def __init__(self):
        self.autoCommand=None
        self.primaryAutoCommand=None
        self.previewedTrajectoryName=None
        self.previewedSecondTrajectoryName=None
        self.previewedFlipForRedAlliance=None
        self.previewedSecondAutoEnabled=None
        self.autoTransitionActive=False
        self.autoTransitionStartTime=None
        self.autoTransitionTargetCommand=None
        self.autoTransitionTargetPoint=None
        self.autoTransitionIndexerStarted=False
        self.autoTransitionLockedSince=None
        self.autoTransitionShooterStarted=False
        self.autoTransitionShooterStartTime=None
        self.autoVelocityEnabled=None
        self.bindingsConfigured=False
        self.matchModeName=None
        self.matchModeStartTime=None

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
        self.initializeAutoVelocityChooser()
        self.initializeMatchTimerDashboard()
        self.syncDashboardControls()
        self.updateTrajectoryPreview(force=True)
        
        #Set default Command (runs over and over)
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        print("containerInited")

    def ensureButtonBindings(self):
        if self.bindingsConfigured:
            return
        self.buttonBindings()
        self.bindingsConfigured=True

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
        wpilib.SmartDashboard.putString("Second Auto Preview Trajectory","Disabled")
        wpilib.SmartDashboard.putString("Second Auto Preview Start Pose","Unavailable")
        wpilib.SmartDashboard.putString("Second Auto Preview End Pose","Unavailable")
        wpilib.SmartDashboard.putString("Second Auto Preview Error","Second auto disabled")
        wpilib.SmartDashboard.putString("Second Auto Preview Indicator","Separate field objects")
        wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")
        wpilib.SmartDashboard.putBoolean("Auto Transition Active",False)

    def initializeAutoVelocityChooser(self):
        self.autoVelocityChooser=wpilib.SendableChooser()
        self.autoVelocityChooser.setDefaultOption("ON",True)
        self.autoVelocityChooser.addOption("OFF",False)
        wpilib.SmartDashboard.putData("Auto Velocity",self.autoVelocityChooser)

    def initializeMatchTimerDashboard(self):
        wpilib.SmartDashboard.putString("Match Mode","Disabled")
        wpilib.SmartDashboard.putString("Match Segment","Disabled")
        wpilib.SmartDashboard.putString("Current Shift","Disabled")
        wpilib.SmartDashboard.putString("Match Time Source","Local")
        wpilib.SmartDashboard.putNumber("Match Time Remaining (s)",0.0)
        wpilib.SmartDashboard.putNumber("Segment Time Remaining (s)",0.0)
        wpilib.SmartDashboard.putNumber("Current Shift Time Remaining (s)",0.0)
        wpilib.SmartDashboard.putNumber("DriverStation Match Time (s)",-1.0)
        wpilib.SmartDashboard.putBoolean("Endgame Active",False)

    def getAutoVelocityEnabled(self):
        selected=self.autoVelocityChooser.getSelected()
        if selected is None:
            return True
        return bool(selected)

    def syncDashboardControls(self):
        selected=self.getAutoVelocityEnabled()
        if self.autoVelocityEnabled != selected:
            self.autoVelocityEnabled=selected
        self.shooterSubsystem.setAutoVelocityEnabled(selected)

    def getCurrentMatchMode(self):
        if wpilib.DriverStation.isAutonomousEnabled():
            return "Autonomous"
        if wpilib.DriverStation.isTeleopEnabled():
            return "Teleop"
        if wpilib.DriverStation.isTestEnabled():
            return "Test"
        if wpilib.DriverStation.isDisabled():
            return "Disabled"
        return "Unknown"

    def getMatchModeDurationSeconds(self,modeName):
        if modeName=="Autonomous":
            return AUTO_PERIOD_SECONDS
        if modeName=="Teleop":
            return TELEOP_PERIOD_SECONDS
        return None

    def getModeRemainingFromLocalTimer(self,now,modeName):
        modeDuration=self.getMatchModeDurationSeconds(modeName)
        if modeDuration is None or self.matchModeStartTime is None:
            return 0.0
        elapsed=max(0.0,now-self.matchModeStartTime)
        return max(0.0,modeDuration-elapsed)

    def getSegmentState(self,modeName,modeRemainingSeconds):
        if modeName=="Autonomous":
            return "Autonomous",modeRemainingSeconds
        if modeName=="Teleop":
            if modeRemainingSeconds > TELEOP_PERIOD_SECONDS - TRANSITION_SHIFT_SECONDS:
                return "Transition Shift",modeRemainingSeconds - (TELEOP_PERIOD_SECONDS - TRANSITION_SHIFT_SECONDS)
            if modeRemainingSeconds > ENDGAME_PERIOD_SECONDS + (ALLIANCE_SHIFT_SECONDS * 3):
                return "Shift 1",modeRemainingSeconds - (ENDGAME_PERIOD_SECONDS + (ALLIANCE_SHIFT_SECONDS * 3))
            if modeRemainingSeconds > ENDGAME_PERIOD_SECONDS + (ALLIANCE_SHIFT_SECONDS * 2):
                return "Shift 2",modeRemainingSeconds - (ENDGAME_PERIOD_SECONDS + (ALLIANCE_SHIFT_SECONDS * 2))
            if modeRemainingSeconds > ENDGAME_PERIOD_SECONDS + ALLIANCE_SHIFT_SECONDS:
                return "Shift 3",modeRemainingSeconds - (ENDGAME_PERIOD_SECONDS + ALLIANCE_SHIFT_SECONDS)
            if modeRemainingSeconds > ENDGAME_PERIOD_SECONDS:
                return "Shift 4",modeRemainingSeconds - ENDGAME_PERIOD_SECONDS
            return "End Game",modeRemainingSeconds
        if modeName=="Test":
            return "Test",0.0
        if modeName=="Disabled":
            return "Disabled",0.0
        return "Unknown",0.0

    def updateMatchTimerDashboard(self):
        now=wpilib.Timer.getFPGATimestamp()
        modeName=self.getCurrentMatchMode()
        if modeName != self.matchModeName:
            self.matchModeName=modeName
            self.matchModeStartTime=now

        driverStationMatchTime=float(wpilib.DriverStation.getMatchTime())
        if self.getMatchModeDurationSeconds(modeName) is not None and driverStationMatchTime >= 0.0:
            modeRemainingSeconds=driverStationMatchTime
            timeSource="DriverStation"
        else:
            modeRemainingSeconds=self.getModeRemainingFromLocalTimer(now,modeName)
            timeSource="Local"

        segmentName,segmentRemainingSeconds=self.getSegmentState(modeName,modeRemainingSeconds)
        wpilib.SmartDashboard.putString("Match Mode",modeName)
        wpilib.SmartDashboard.putString("Match Segment",segmentName)
        wpilib.SmartDashboard.putString("Current Shift",segmentName)
        wpilib.SmartDashboard.putString("Match Time Source",timeSource)
        wpilib.SmartDashboard.putNumber("Match Time Remaining (s)",modeRemainingSeconds)
        wpilib.SmartDashboard.putNumber("Segment Time Remaining (s)",segmentRemainingSeconds)
        wpilib.SmartDashboard.putNumber("Current Shift Time Remaining (s)",segmentRemainingSeconds)
        wpilib.SmartDashboard.putNumber("DriverStation Match Time (s)",driverStationMatchTime)
        wpilib.SmartDashboard.putBoolean("Endgame Active",segmentName=="End Game")

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

    def clearPreviewObjects(self,pathObject,startObject,endObject):
        pathObject.setPoses([])
        startObject.setPoses([])
        endObject.setPoses([])

    def setPreviewStatus(self,prefix,trajectoryName,startPose,endPose,errorMessage):
        wpilib.SmartDashboard.putString(f"{prefix} Preview Trajectory",trajectoryName)
        wpilib.SmartDashboard.putString(f"{prefix} Preview Start Pose",startPose)
        wpilib.SmartDashboard.putString(f"{prefix} Preview End Pose",endPose)
        wpilib.SmartDashboard.putString(f"{prefix} Preview Error",errorMessage)

    def updateSingleTrajectoryPreview(self,prefix,selectedTrajectoryName,flipForRedAlliance,pathObject,startObject,endObject):
        if not selectedTrajectoryName:
            self.clearPreviewObjects(pathObject,startObject,endObject)
            self.setPreviewStatus(prefix,"None","Unavailable","Unavailable","No trajectory selected")
            return

        try:
            trajectory=choreo.load_swerve_trajectory(selectedTrajectoryName)
            pathPoses=[sample.flipped().get_pose() if flipForRedAlliance else sample.get_pose() for sample in trajectory.samples]
            initialPose=trajectory.get_initial_pose(flipForRedAlliance)
            finalPose=trajectory.get_final_pose(flipForRedAlliance)

            pathObject.setPoses(pathPoses)
            if initialPose is not None:
                startObject.setPose(initialPose)
                startPoseLabel=f"{initialPose.X():.3f}, {initialPose.Y():.3f}, {initialPose.rotation().degrees():.1f}"
            else:
                startObject.setPoses([])
                startPoseLabel="Unavailable"

            if finalPose is not None:
                endObject.setPose(finalPose)
                endPoseLabel=f"{finalPose.X():.3f}, {finalPose.Y():.3f}, {finalPose.rotation().degrees():.1f}"
            else:
                endObject.setPoses([])
                endPoseLabel="Unavailable"

            self.setPreviewStatus(prefix,selectedTrajectoryName,startPoseLabel,endPoseLabel,"")
        except Exception as ex:
            self.clearPreviewObjects(pathObject,startObject,endObject)
            self.setPreviewStatus(prefix,selectedTrajectoryName,"Unavailable","Unavailable",str(ex))

    def updateTrajectoryPreview(self,force=False):
        selectedTrajectoryName=self.getSelectedTrajectoryName()
        selectedSecondTrajectoryName=self.getSelectedSecondTrajectoryName()
        secondAutoEnabled=self.isSecondAutoEnabled()
        flipForRedAlliance=self.shouldFlipTrajectoryForAlliance()
        if not force and selectedTrajectoryName==self.previewedTrajectoryName and selectedSecondTrajectoryName==self.previewedSecondTrajectoryName and secondAutoEnabled==self.previewedSecondAutoEnabled and flipForRedAlliance==self.previewedFlipForRedAlliance:
            return

        self.previewedTrajectoryName=selectedTrajectoryName
        self.previewedSecondTrajectoryName=selectedSecondTrajectoryName
        self.previewedFlipForRedAlliance=flipForRedAlliance
        self.previewedSecondAutoEnabled=secondAutoEnabled
        previewPathObject=self.driveSubsystem.field.getObject("Auto Preview Path")
        previewStartObject=self.driveSubsystem.field.getObject("Auto Preview Start")
        previewEndObject=self.driveSubsystem.field.getObject("Auto Preview End")
        secondPreviewPathObject=self.driveSubsystem.field.getObject("Second Auto Preview Path")
        secondPreviewStartObject=self.driveSubsystem.field.getObject("Second Auto Preview Start")
        secondPreviewEndObject=self.driveSubsystem.field.getObject("Second Auto Preview End")
        wpilib.SmartDashboard.putString("Auto Preview Alliance",self.getTrajectoryAllianceLabel())
        self.updateSingleTrajectoryPreview("Auto",selectedTrajectoryName,flipForRedAlliance,previewPathObject,previewStartObject,previewEndObject)

        if secondAutoEnabled:
            self.updateSingleTrajectoryPreview("Second Auto",selectedSecondTrajectoryName,flipForRedAlliance,secondPreviewPathObject,secondPreviewStartObject,secondPreviewEndObject)
        else:
            self.clearPreviewObjects(secondPreviewPathObject,secondPreviewStartObject,secondPreviewEndObject)
            self.setPreviewStatus("Second Auto","Disabled","Unavailable","Unavailable","Second auto disabled")

    def teleopInit(self):
        self.cancelAutonomousCommand()
        self.setAutoTransitionActive(False)
        self.syncDashboardControls()
        self.driveSubsystem.setDefaultCommand(driveTrainCommand(self.driveSubsystem,self.joystick))
        self.shooterSubsystem.teleopInit()
        self.indexerSubsystem.teleopInit()
        self.intakeSubsystem.teleopInit()
        wpilib.SmartDashboard.putString("Auto Transition Status","Disabled")
        self.flipSubsystem.teleopInit()
        print('entering teleop')

    def autoInit(self):
        self.cancelAutonomousCommand()
        self.syncDashboardControls()
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
        self.setAutoTransitionActive(False)
        self.syncDashboardControls()
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

    def setAutoTransitionActive(self,isActive):
        self.autoTransitionActive=isActive
        wpilib.SmartDashboard.putBoolean("Auto Transition Active",isActive)

    def getAutoTransitionAimErrorDegrees(self):
        if self.autoTransitionTargetPoint is None:
            return None
        robotPose=self.driveSubsystem.getPoseState()
        targetX,targetY=self.autoTransitionTargetPoint
        return getTargetAimErrorDegrees(robotPose,targetX,targetY)

    def resetAutoTransitionShotState(self):
        self.autoTransitionIndexerStarted=False
        self.autoTransitionLockedSince=None
        self.autoTransitionShooterStarted=False
        self.autoTransitionShooterStartTime=None

    def shouldStartAutoTransitionIndexer(self):
        if self.autoTransitionTargetPoint is None:
            return False
        robotPose=self.driveSubsystem.getPoseState()
        targetX,targetY=self.autoTransitionTargetPoint
        if not isTargetLocked(robotPose,targetX,targetY,auxiliaryConfig.autoTargetAimToleranceDegrees):
            self.autoTransitionLockedSince=None
            return False
        now=wpilib.Timer.getFPGATimestamp()
        if self.autoTransitionLockedSince is None:
            self.autoTransitionLockedSince=now
            return False
        if not self.autoTransitionShooterStarted:
            if now-self.autoTransitionLockedSince >= auxiliaryConfig.autoTargetLockHoldSeconds:
                self.shooterSubsystem.autoShootStart()
                self.autoTransitionShooterStarted=True
                self.autoTransitionShooterStartTime=now
                wpilib.SmartDashboard.putString("Auto Transition Status","Spinning Up")
            return False
        if self.autoTransitionShooterStartTime is None:
            return False
        if now-self.autoTransitionLockedSince < auxiliaryConfig.autoTargetLockHoldSeconds:
            return False
        return now-self.autoTransitionShooterStartTime >= auxiliaryConfig.shooterStartupTime

    def beginAutoTransition(self):
        self.setAutoTransitionActive(True)
        self.autoTransitionStartTime=wpilib.Timer.getFPGATimestamp()
        self.shooterSubsystem.autoInit()
        self.indexerSubsystem.autoInit()
        targetX,targetY=getSpeakerTargetPoint()
        self.autoTransitionTargetPoint=(targetX,targetY)
        self.autoTransitionTargetCommand=targetPointCommand(self.driveSubsystem,targetX,targetY)
        self.resetAutoTransitionShotState()
        self.driveSubsystem.overideInput()
        if self.autoTransitionTargetCommand is not None:
            self.autoTransitionTargetCommand.execute()
        self.driveSubsystem.setState(0,0,0)
        self.intakeSubsystem.autoGrabStop()
        wpilib.SmartDashboard.putString("Auto Transition Status","Aiming")

    def executeAutoTransition(self):
        if self.autoTransitionTargetCommand is not None:
            self.autoTransitionTargetCommand.execute()
        self.driveSubsystem.setState(0,0,0)
        robotPose=self.driveSubsystem.getPoseState()
        self.shooterSubsystem.setTargetDistance(getSpeakerDistanceMeters(robotPose))
        if self.autoTransitionShooterStarted and self.autoTransitionShooterStartTime is not None and wpilib.Timer.getFPGATimestamp() >= self.autoTransitionShooterStartTime + auxiliaryConfig.autoShootStartToIntakeUpDelaySeconds:
            self.intakeSubsystem.autoIntakeUp()
        if not self.autoTransitionIndexerStarted and self.shouldStartAutoTransitionIndexer():
            self.shooterSubsystem.autoFeedStart()
            self.indexerSubsystem.autoFeedStart()
            self.autoTransitionIndexerStarted=True
            wpilib.SmartDashboard.putString("Auto Transition Status","Launching")
        elif self.autoTransitionIndexerStarted:
            targetX,targetY=self.autoTransitionTargetPoint
            if not isTargetLocked(robotPose,targetX,targetY,auxiliaryConfig.autoTargetAimToleranceDegrees):
                self.autoTransitionLockedSince=None
                self.shooterSubsystem.autoFeedStop()
                self.indexerSubsystem.autoFeedStop()
                self.autoTransitionIndexerStarted=False
                wpilib.SmartDashboard.putString("Auto Transition Status","Reacquiring")

    def finishAutoTransition(self):
        self.setAutoTransitionActive(False)
        self.autoTransitionStartTime=None
        self.resetAutoTransitionShotState()
        self.autoTransitionTargetPoint=None
        if self.autoTransitionTargetCommand is not None:
            self.autoTransitionTargetCommand.end(interrupted=True)
            self.autoTransitionTargetCommand=None
        self.driveSubsystem.overideInput()
        self.shooterSubsystem.autoFeedStop()
        self.indexerSubsystem.autoFeedStop()
        self.shooterSubsystem.autoShootStop()
        self.indexerSubsystem.autoShootStop()
        self.intakeSubsystem.autoGrabStop()
        wpilib.SmartDashboard.putString("Auto Transition Status","Complete")

    def buildAutoTransitionCommand(self):
        return commands2.FunctionalCommand(
            self.beginAutoTransition,
            self.executeAutoTransition,
            lambda interrupted: self.finishAutoTransition(),
            lambda: False,
            self.driveSubsystem,
        ).withTimeout(auxiliaryConfig.autoTransitionDelaySeconds)

    def buildAutonomousCommand(self):
        selectedTrajectoryName=self.getSelectedTrajectoryName()
        selectedSecondTrajectoryName=self.getSelectedSecondTrajectoryName()
        self.primaryAutoCommand=self.createAutoDriveCommand(selectedTrajectoryName)
        autoCommands=[self.primaryAutoCommand]
        hasEnabledSecondAuto=self.isSecondAutoEnabled() and selectedSecondTrajectoryName

        if hasEnabledSecondAuto:
            autoCommands.append(self.buildAutoTransitionCommand())
            autoCommands.append(self.createAutoDriveCommand(selectedSecondTrajectoryName))

        if selectedTrajectoryName or hasEnabledSecondAuto:
            autoCommands.append(self.buildAutoTransitionCommand())
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
            self.controller.button(swerveConfig.targetingButton).whileTrue(targetPointCommand(self.driveSubsystem))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
            self.controller.button(3).whileTrue(setSwerveXMode(self.driveSubsystem))
        if swerveConfig.driveController=="VKBJoystick":
            self.controller.button(15).whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.button(3).whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem)))
            self.controller.button(1).whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        if swerveConfig.driveController=="XboxController":
            self.controller.a().whileTrue(fieldOrientReorient(self.driveSubsystem))
            self.controller.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,11.91497, 4.03514)))
            self.controller.rightTrigger().whileTrue(commands2.RepeatCommand(pointToVelocityVectorCommand(self.driveSubsystem,self.joystick)))
        self.auxController.rightTrigger().whileTrue(commands2.RepeatCommand(shooterDistanceCommand.setShooterDistanceCommand(self.driveSubsystem,self.shooterSubsystem)))
        # if True:
        #     self.auxController.povUp().whileTrue(flipSubsys.flipGrabberCommand(self.flipSubsystem,-0.5))
        #     self.auxController.povDown().whileTrue(flipSubsys.flipGrabberCommand(self.flipSubsystem,0.1))
        #     self.auxController.povLeft().whileTrue(flipSubsys.flipTrackCommand(self.flipSubsystem,0.5))
        #     self.auxController.povRight().whileTrue(flipSubsys.flipTrackCommand(self.flipSubsystem,-0.5))

        pass

        ##Shooter bindings
        #if auxiliaryConfig.auxController=="XboxController":
            #self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
           # self.auxController.x().whileTrue(commands2.RepeatCommand(targetPointCommand(self.driveSubsystem,4.62507, 4.03514)))
            #self.auxController.y().whileTrue(commands2.RepeatCommand(overideRobotInput(self.driveSubsystem,theta=0.1)))
            #pass
    def testInit(self):
        self.flipSubsystem.testInit()
        pass

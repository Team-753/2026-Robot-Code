import commands2,wpimath,wpimath.controller,wpimath.trajectory,wpimath.kinematics,wpilib
from math import pi
from Drivetrain.swerveSubsys import driveTrainSubsys,pointToVelocityVectorCommand
from Drivetrain.Targeting2 import getSpeakerDistanceMeters,isSpeakerLocked,targetPointCommand
from AuxilarySystems import auxiliaryConfig,shooterSubsys,IntakeSubsys,IndexerSubsys
import choreo


def _shouldFlipTrajectoryForAlliance():
    alliance=wpilib.DriverStation.getAlliance()
    return alliance == wpilib.DriverStation.Alliance.kRed


class autoDriveTrainCommand(commands2.Command):
    def __init__(self,shooterSubsys:shooterSubsys.shooterSubsys,intakeSubsys:IntakeSubsys.intakeSubsys,indexerSubsys:IndexerSubsys.indexerSubsys,driveSubsys:driveTrainSubsys,trajectoryName:str=""):
        super().__init__()
        self.eventList=[]
        self.nextEventIndex=0
        #SUBSYS
        self.driveSubsys=driveSubsys
        self.shooterSubsys=shooterSubsys
        self.intakeSubsys=intakeSubsys
        self.indexerSubsys=indexerSubsys
        self.traj=None
        self.initialPose=None
        self.totalTime=0.0
        self.flipForRedAlliance=_shouldFlipTrajectoryForAlliance()
        self.addRequirements(driveSubsys)
        self.autoTargetCommand=None
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        ##AUX VARS
        self.shooterState=False
        self.intakeDown=True
        self.intakeSpin=False
        self.autoShotLockedSince=None
        self.autoShotSpinStartTime=None
        self.autoShotShooterStarted=False
        self.autoShotFeedStarted=False
        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(20,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(2.5,0.1,0),cont.PIDController(2.5,0.1,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.xPid=cont.PIDController(12,0.05,0.1)
        self.yPid=cont.PIDController(12,0.05,0.1)
        self.omegaPid=cont.ProfiledPIDControllerRadians(16,0.0002,0.3,wpimath.trajectory.TrapezoidProfileRadians.Constraints(6*pi,6*pi))
        self.omegaPid.enableContinuousInput(-pi,pi)
        # Load the selected Choreo path once so auto can fail safe if the selection is missing.
        if trajectoryName:
            try:
                self.traj=choreo.load_swerve_trajectory(trajectoryName)
                self.initialPose=self.traj.get_initial_pose(self.flipForRedAlliance)
                self.totalTime=self.traj.get_total_time()
                for eventMarker in self.traj.events:
                    self.eventList.append((eventMarker.timestamp,eventMarker.event))
                wpilib.SmartDashboard.putString("Auto Trajectory Loaded",trajectoryName)
                wpilib.SmartDashboard.putString("Auto Trajectory Alliance","Red" if self.flipForRedAlliance else "Blue")
                wpilib.SmartDashboard.putString("Auto Trajectory Error","")
            except Exception as ex:
                self.traj=None
                self.initialPose=None
                self.eventList=[]
                wpilib.SmartDashboard.putString("Auto Trajectory Loaded","None")
                wpilib.SmartDashboard.putString("Auto Trajectory Alliance","Unknown")
                wpilib.SmartDashboard.putString("Auto Trajectory Error",str(ex))
        else:
            wpilib.SmartDashboard.putString("Auto Trajectory Loaded","None")
            wpilib.SmartDashboard.putString("Auto Trajectory Alliance","Unknown")
            wpilib.SmartDashboard.putString("Auto Trajectory Error","")
        self.clock=wpilib.Timer()

    def initialize(self):
        self.shooterSubsys.autoInit()
        self.indexerSubsys.autoInit()
        self.resetAutoShotSequence()
        self.clock.reset()
        self.clock.start()
        self.nextEventIndex=0
        if self.traj is not None:
            print(self.traj.events)

    def getInitialPose(self):
        return self.initialPose

    def getSpeeds(self, sample):
        # Get the current pose of the robot
        pose = self.driveSubsys.getPoseState()
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            offset=pi
        else:
            offset=0
        # Generate the next speeds for the robot
        speeds = wpimath.kinematics.ChassisSpeeds(
            sample.vx + self.xPid.calculate(pose.X(), sample.x),
            sample.vy + self.yPid.calculate(pose.Y(), sample.y),
            sample.omega + self.omegaPid.calculate(wpimath.geometry.Rotation2d(pose.rotation().radians()+offset).radians(), sample.heading)
        )
        return speeds

    def resetAutoShotSequence(self):
        self.autoShotLockedSince=None
        self.autoShotSpinStartTime=None
        self.autoShotShooterStarted=False
        self.autoShotFeedStarted=False

    def stopAutoShotSequence(self):
        if self.autoShotShooterStarted or self.autoShotFeedStarted or self.shooterSubsys.isShooting() or self.indexerSubsys.autoFeedActive:
            self.shooterSubsys.autoFeedStop()
            self.indexerSubsys.autoFeedStop()
            self.shooterSubsys.autoShootStop()
            self.indexerSubsys.autoShootStop()
        self.resetAutoShotSequence()

    def executeAutoShotSequence(self):
        if self.autoTargetCommand is None:
            self.autoTargetCommand=targetPointCommand(self.driveSubsys)

        robotPose=self.driveSubsys.getPoseState()
        now=wpilib.Timer.getFPGATimestamp()
        self.shooterSubsys.setTargetDistance(getSpeakerDistanceMeters(robotPose))
        self.autoTargetCommand.execute()

        locked=isSpeakerLocked(robotPose,auxiliaryConfig.autoTargetAimToleranceDegrees)
        if locked:
            if self.autoShotLockedSince is None:
                self.autoShotLockedSince=now
        else:
            self.autoShotLockedSince=None
            if self.autoShotFeedStarted:
                self.shooterSubsys.autoFeedStop()
                self.indexerSubsys.autoFeedStop()
                self.autoShotFeedStarted=False

        if not self.autoShotShooterStarted and self.autoShotLockedSince is not None:
            if now-self.autoShotLockedSince >= auxiliaryConfig.autoTargetLockHoldSeconds:
                self.shooterSubsys.autoShootStart()
                self.autoShotShooterStarted=True
                self.autoShotSpinStartTime=now

        if self.autoShotShooterStarted and locked and not self.autoShotFeedStarted and self.autoShotSpinStartTime is not None and self.autoShotLockedSince is not None:
            if now-self.autoShotLockedSince >= auxiliaryConfig.autoTargetLockHoldSeconds and now-self.autoShotSpinStartTime >= auxiliaryConfig.shooterStartupTime:
                self.shooterSubsys.autoFeedStart()
                self.indexerSubsys.autoFeedStart()
                self.autoShotFeedStarted=True

    def execute(self):
        if self.traj is None:
            self.driveSubsys.setState(0,0,0)
            return
        while self.nextEventIndex < len(self.eventList) and self.clock.get() >= self.eventList[self.nextEventIndex][0]:
            exec("self."+str(self.eventList[self.nextEventIndex][1]))
            self.nextEventIndex+=1
        self.goal=self.traj.sample_at(self.clock.get(),self.flipForRedAlliance)
        speeds=self.getSpeeds(self.goal)
        #NOTE EXPLAIN LATER
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.driveSubsys.setState(-speeds.vx,speeds.vy,speeds.omega)
        else:
            self.driveSubsys.setState(speeds.vx,-speeds.vy,speeds.omega)

        #LIST OF COMMANDS
        #setShooting(bool)
        #setIntakeDown(bool)
        #setIntakeSpin(bool)
        #setPointVV(bool)
        if self.shooterState:
            self.executeAutoShotSequence()
        else:
            if self.autoTargetCommand is not None:
                self.autoTargetCommand.end(interrupted=True)
                self.autoTargetCommand=None
            self.stopAutoShotSequence()
        
        if self.intakeDown:
            self.intakeSubsys.autoIntakeDown()
        else:
            self.intakeSubsys.autoIntakeUp()
        
        if self.intakeSpin:
            self.intakeSubsys.autoGrabStart()
        else:
            self.intakeSubsys.autoGrabStop()

    def end(self,interrupted):
        self.clock.stop()
        if self.autoTargetCommand is not None:
            self.autoTargetCommand.end(interrupted=True)
            self.autoTargetCommand=None
        self.driveSubsys.overideInput()
        self.driveSubsys.setState(0,0,0)
        self.stopAutoShotSequence()
        self.intakeSubsys.autoGrabStop()

    def isFinished(self):
        if self.traj is None:
            return True
        return self.clock.get() >= self.totalTime

    def setShooting(self,shooterState):
        self.shooterState=shooterState
    def setIntakeDown(self,intakeDown):
        self.intakeDown=intakeDown
    def setIntakeSpin(self,intakeSpinning):
        self.intakeSpin=intakeSpinning
    def setIntake(self,intakeSpinning):
        self.setIntakeSpin(intakeSpinning)
    def setIntakeSping(self,intakeSpinning):
        self.setIntakeSpin(intakeSpinning)

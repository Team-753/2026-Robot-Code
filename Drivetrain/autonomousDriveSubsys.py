import commands2,wpimath,wpimath.controller,wpimath.trajectory,wpimath.kinematics,wpilib
from math import pi
from Drivetrain.swerveSubsys import driveTrainSubsys,pointToVelocityVectorCommand
from Drivetrain.Targeting2 import targetPointCommand,targetPointWithLeadCommand
from AuxilarySystems import shooterSubsys,IntakeSubsys,IndexerSubsys
import choreo


def _shouldFlipTrajectoryForAlliance():
    alliance=wpilib.DriverStation.getAlliance()
    return alliance == wpilib.DriverStation.Alliance.kRed


class autoDriveTrainCommand(commands2.Command):
    def __init__(self,shooterSubsys:shooterSubsys.shooterSubsys,intakeSubsys:IntakeSubsys.intakeSubsys,indexerSubsys:IndexerSubsys.indexerSubsys,driveSubsys:driveTrainSubsys,trajectoryName:str=""):
        super().__init__()
        self.eventList=[]
        #SUBSYS
        self.driveSubsys=driveSubsys
        self.shooterSubsys=shooterSubsys
        self.intakeSubsys=intakeSubsys
        self.indexerSubsys=indexerSubsys
        self.traj=None
        self.initialPose=None
        self.flipForRedAlliance=_shouldFlipTrajectoryForAlliance()
        self.addRequirements(driveSubsys)
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        ##AUX VARS
        self.shooterState=False
        self.intakeDown=True
        self.intakeSpin=False
        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(20,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(2.5,0.1,0),cont.PIDController(2.5,0.1,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.xPid=cont.PIDController(12,0.05,0.1)
        self.yPid=cont.PIDController(12,0.05,0.1)
        self.omegaPid=cont.ProfiledPIDControllerRadians(14,0.00001,0.3,wpimath.trajectory.TrapezoidProfileRadians.Constraints(6*pi,6*pi))
        # Load the selected Choreo path once so auto can fail safe if the selection is missing.
        if trajectoryName:
            try:
                self.traj=choreo.load_swerve_trajectory(trajectoryName)
                self.initialPose=self.traj.get_initial_pose(self.flipForRedAlliance)
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
        self.clock.reset()
        self.clock.start()
        if self.traj is not None:
            print(self.traj.events)

    def getInitialPose(self):
        return self.initialPose

    def getSpeeds(self, sample):
        # Get the current pose of the robot
        pose = self.driveSubsys.getPoseState()

        # Generate the next speeds for the robot
        speeds = wpimath.kinematics.ChassisSpeeds(
            sample.vx + self.xPid.calculate(pose.X(), sample.x),
            sample.vy + self.yPid.calculate(pose.Y(), sample.y),
            sample.omega + self.omegaPid.calculate(pose.rotation().radians(), sample.heading)
        )
        return speeds
    def execute(self):
        if self.traj is None:
            self.driveSubsys.setState(0,0,0)
            return
        for i in range(len(self.eventList)):
            if self.clock.get()>self.eventList[i][0]:
                exec("self."+str(self.eventList[i][1]))
        self.goal=self.traj.sample_at(self.clock.get(),self.flipForRedAlliance)
        speeds=self.getSpeeds(self.goal)
        #NOTE EXPLAIN LATER
<<<<<<< HEAD
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.driveSubsys.setState(-speeds.vx,speeds.vy,speeds.omega)
=======
        if wpilib.DriverStation.getAlliance()==wpilib.DriverStation.Alliance.kRed:
            self.driveSubsys.setState(speeds.vx,-speeds.vy,speeds.omega)
>>>>>>> a60830d104a9b3707e6dbb093cdb3a5dc4df88eb
        else:
            self.driveSubsys.setState(speeds.vx,-speeds.vy,speeds.omega)

        #LIST OF COMMANDS
        #setShooting(bool)
        #setIntakeDown(bool)
        #setIntakeSpin(bool)
        #setPointVV(bool)
        if self.shooterState:
            #targetPointCommand(self.driveSubsys,1,1).asProxy()
            #commands2.InstantCommand(targetPointCommand(self.driveSubsys,1,1))
            #commands2.CommandScheduler.schedule(commands2.CommandScheduler.getInstance(),commands2.RepeatCommand(targetPointWithLeadCommand(self.driveSubsys)))
            val=targetPointWithLeadCommand(self.driveSubsys).execute()
            self.shooterSubsys.autoShootStart()
            self.indexerSubsys.autoShootStart()
            #commands2.RepeatCommand(targetPointCommand(self.driveSubsys,1,1))
        else:
            targetPointWithLeadCommand(self.driveSubsys).end(interrupted=True)
            self.shooterSubsys.autoShootStop()
            self.indexerSubsys.autoShootStop()
        
        if self.intakeDown:
            self.intakeSubsys.autoIntakeDown()
        else:
            self.intakeSubsys.autoIntakeUp()
        
        if self.intakeSpin:
            self.intakeSubsys.autoGrabStart()
        else:
            self.intakeSubsys.autoGrabStop()

    def setShooting(self,shooterState):
        self.shooterState=shooterState
    def setIntakeDown(self,intakeDown):
        self.intakeDown=intakeDown
    def setIntakeSpin(self,intakeSpinning):
        self.intakeSpin=intakeSpinning

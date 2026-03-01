import commands2,wpimath,wpimath.controller,wpimath.trajectory,wpimath.kinematics,wpilib
from math import pi
from Drivetrain.swerveSubsys import driveTrainSubsys
import choreo
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,trajectoryName:str=""):
        super().__init__()
        self.eventList=[]
        self.driveSubsys=driveSubsys
        self.traj=None
        self.initialPose=None
        self.addRequirements(driveSubsys)
        cont=wpimath.controller
        wpigeo=wpimath.geometry

        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(20,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(2.5,0.1,0),cont.PIDController(2.5,0.1,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.xPid=cont.PIDController(4,0.05,0)
        self.yPid=cont.PIDController(4,0.05,0)
        self.omegaPid=cont.ProfiledPIDControllerRadians(8,0.2,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        # Load the selected Choreo path once so auto can fail safe if the selection is missing.
        if trajectoryName:
            try:
                self.traj=choreo.load_swerve_trajectory(trajectoryName)
                startSample=self.traj.sample_at(0)
                self.initialPose=wpigeo.Pose2d(
                    startSample.x,
                    startSample.y,
                    wpigeo.Rotation2d(startSample.heading),
                )
                for eventMarker in self.traj.events:
                    self.eventList.append((eventMarker.timestamp,eventMarker.event))
                wpilib.SmartDashboard.putString("Auto Trajectory Loaded",trajectoryName)
                wpilib.SmartDashboard.putString("Auto Trajectory Error","")
            except Exception as ex:
                self.traj=None
                self.initialPose=None
                self.eventList=[]
                wpilib.SmartDashboard.putString("Auto Trajectory Loaded","None")
                wpilib.SmartDashboard.putString("Auto Trajectory Error",str(ex))
        else:
            wpilib.SmartDashboard.putString("Auto Trajectory Loaded","None")
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
        for i in self.eventList:
            if self.clock.get()>i[0]:
                print(i[0])
        self.goal=self.traj.sample_at(self.clock.get())
        speeds=self.getSpeeds(self.goal)
        #print(self.driveSubsys.getPoseState())#,self.driveSubsys.getPoseState().y_feet)
        #print(self.driveSubsys.getPoseState())
        self.driveSubsys.setState(-speeds.vx,speeds.vy,speeds.omega)

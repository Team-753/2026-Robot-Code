import commands2,wpimath,wpimath.controller,wpimath.trajectory,wpimath.kinematics,wpilib
from math import pi
from Drivetrain.swerveSubsys import driveTrainSubsys
import choreo
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        super().__init__()

        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(20,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(2.5,0.1,0),cont.PIDController(2.5,0.1,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.xPid=cont.PIDController(4,0.05,0)
        self.yPid=cont.PIDController(4,0.05,0)
        self.omegaPid=cont.ProfiledPIDControllerRadians(8,0.2,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.traj=choreo.load_swerve_trajectory("Path2")
        self.clock=wpilib.Timer()
        self.clock.start()
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
        self.goal=self.traj.sample_at(self.clock.get())
        #speeds=self.holoCont.calculate(self.driveSubsys.getPoseState(),self.goal.get_pose(),wpimath.geometry.Rotation2d.fromDegrees(self.goal.heading))
        speeds=self.getSpeeds(self.goal)
        #print(self.driveSubsys.getPoseState())#,self.driveSubsys.getPoseState().y_feet)
        print(self.driveSubsys.getPoseState())
        self.driveSubsys.setState(-speeds.vx,speeds.vy,speeds.omega)
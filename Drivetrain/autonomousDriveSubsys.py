import commands2,wpimath,wpimath.controller,wpimath.trajectory,wpilib
from math import pi
from Drivetrain.swerveSubsys import driveTrainSubsys
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        super().__init__()
        config = wpimath.trajectory.TrajectoryConfig(1,1)

        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(20,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(2.5,0.1,0),cont.PIDController(2.5,0.1,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.trajectory=wpimath.trajectory.TrajectoryGenerator.generateTrajectory([startPos,endPos],config=config)
        self.clock=wpilib.Timer()
        self.clock.start()
    def execute(self):
        self.goal=self.trajectory.sample(self.clock.get())
        speeds=self.holoCont.calculate(self.driveSubsys.getPoseState(),self.goal,wpimath.geometry.Rotation2d(0))
        #print(self.driveSubsys.getPoseState())#,self.driveSubsys.getPoseState().y_feet)
        print(speeds.vx,speeds.vy)
        self.driveSubsys.setState(-speeds.vx,speeds.vy,speeds.omega)
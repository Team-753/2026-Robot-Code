import commands2,wpimath.controller,wpimath.trajectory
from math import atan2,pi
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
from wpilib import Timer
        
class targetPointCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.TARGET_POINT_RED = (4.62507, 4.03514)
        self.TARGET_POINT_BLUE = (11.91497, 4.03514)
        self.driveSubsys=driveSubsys
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(65,0.06,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.currentTime = Timer.getFPGATimestamp()
        
    def _get_alliance(self):
        try:
            alliance = wpilib.DriverStation.getAlliance()
        except Exception:
            return None
        return alliance

    def _get_target_point(self):
        alliance = self._get_alliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return self.TARGET_POINT_RED
        return self.TARGET_POINT_BLUE

    def timestampPose(self):
        return (self.driveSubsys.getPoseState,self.currentTime)

    def calucateVelocity(self):
        pass

    def execute(self):
        robotPose=self.driveSubsys.getPoseState()
        desiredRotation=atan2(self._get_target_point)
        output=self.thetaPid.calculate(robotPose.rotation().radians(),desiredRotation)
        #print(robotPose.rotation().radians(),desiredRotation,self.ty-robotPose.y)
        self.driveSubsys.overideInput(rot=output)
        print(self.timestampPose)
        print("Sending Time Data")
    def end(self,interrupted):
        self.driveSubsys.overideInput()

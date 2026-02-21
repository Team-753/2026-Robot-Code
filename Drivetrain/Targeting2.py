import commands2,wpimath.controller,wpimath.trajectory
from math import atan2,pi
import wpilib
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
from wpilib import Timer
import wpilib
        
#NOTICE: This targeting system assumes the metric system. 

#Unfortunantly, Cheeseburgers-per-hour was not easy to implement into the code (plus I am lazy) -Ryan T
class targetPointCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.TARGET_POINT_BLUE = (11.91497, 4.03514)
        self.TARGET_POINT_RED = (4.62507,4.03514)
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(65,0.06,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.currentTime = Timer.getFPGATimestamp()
        self.pastPosition = None
        self.velx,self.vely=0,0
    def _get_alliance(self): #We need this information so that we dont score in the wrong hub.
        try:
            alliance = wpilib.DriverStation.getAlliance()
        except Exception:
            return None
        return alliance

    def _get_target_point(self): #Sets the target point based on the team we are on. 
        alliance = self._get_alliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return [self.TARGET_POINT_RED[0]-self.velx,self.TARGET_POINT_RED[1]-self.vely]
        return [self.TARGET_POINT_BLUE[0]-self.velx,self.TARGET_POINT_BLUE[1]-self.vely]

    def timestampPose(self):#Used to timestamp where the robot is so that we can run velocity calulations. 
        return (self.driveSubsys.getPoseState,self.currentTime)

    def calucateVelocity(self): #Used to calculate the velocity of the robot so that we can adjust the target point
        pass

    def adjustedTargetPoint(self): #Returns the point we are looking for. 
        pass

    def execute(self):
        if robotPose:
            self.pastPosition=robotPose.getPoseState()
            self.pastTime=Timer.getFPGATimestamp()
            print(Timer.getFPGATimestamp)
        robotPose=self.driveSubsys.getPoseState()
        if pastPosition:
            self.velx=self.pastPosition.x
            self.vely=self.pastPosition.y
        desiredRotation=atan2(self._get_target_point)
        output=self.thetaPid.calculate(robotPose.rotation().radians(),desiredRotation)
        #print(robotPose.rotation().radians(),desiredRotation,self.ty-robotPose.y)
        self.driveSubsys.overideInput(rot=output)
        print(self.timestampPose)
        print("Sending Time Data")
    def end(self,interrupted):
        self.driveSubsys.overideInput()

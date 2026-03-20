import commands2,wpimath.controller,wpimath.trajectory
from math import atan2,pi,sqrt
import wpilib
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
from wpilib import Timer
import wpilib
import AuxilarySystems.shooterSubsys
from customFunctions import pythag
#NOTICE: This targeting system assumes the metric system. 
#Unfortunantly, Cheeseburgers-per-hour was not easy to implement into the code (plus I am lazy) -Ryan T

TARGET_POINT_BLUE = (4.62507, 4.03514)
TARGET_POINT_RED = (11.91497, 4.03514)
class setShooterDistanceCommand(commands2.Command): #This class is used for estimating where the robot needs to look based on its velocity. 
    def __init__(self,driveSubsys:driveTrainSubsys,shootSubsys:AuxilarySystems.shooterSubsys.shooterSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.shooterSubsys= shootSubsys
        self.dt=driveSubsys
        self.TARGET_POINT_BLUE = TARGET_POINT_BLUE
        self.TARGET_POINT_RED = TARGET_POINT_RED
    def _get_alliance(self): #We need this information so that we dont score in the wrong hub.
        try:
            alliance = wpilib.DriverStation.getAlliance()
        except Exception:
            print("getAllianceError")
            return None
        return alliance

    def _get_target_point(self): #Sets the target point based on the team we are on. 
        alliance = self._get_alliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return [self.TARGET_POINT_RED[0],self.TARGET_POINT_RED[1]]
        return [self.TARGET_POINT_BLUE[0],self.TARGET_POINT_BLUE[1]]
    def execute(self):
        self.robotPose=self.dt.getPoseState()
        distance=pythag(self.robotPose.x,self.robotPose.y,self._get_target_point()[0],self._get_target_point()[1])
        self.shooterSubsys.setTargetDistance(distance)
    def end(self,interrupted):
        self.driveSubsys.overideInput()

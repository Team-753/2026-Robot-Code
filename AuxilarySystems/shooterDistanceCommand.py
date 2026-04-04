import commands2
import wpilib

import AuxilarySystems.shooterSubsys
from Drivetrain.swerveSubsys import driveTrainSubsys
from customFunctions import pythag
#NOTICE: This targeting system assumes the metric system. 
#Unfortunantly, Cheeseburgers-per-hour was not easy to implement into the code (plus I am lazy) -Ryan T

TARGET_POINT_BLUE = (4.62507, 4.03514)
TARGET_POINT_RED = (11.91497, 4.03514)


def getSpeakerTargetPoint():
    alliance = wpilib.DriverStation.getAlliance()
    if alliance == wpilib.DriverStation.Alliance.kRed:
        return TARGET_POINT_RED
    return TARGET_POINT_BLUE


def updateShooterDistanceFromDrivePose(driveSubsys:driveTrainSubsys,shootSubsys:AuxilarySystems.shooterSubsys.shooterSubsys):
    robotPose=driveSubsys.getPoseState()
    targetX,targetY=getSpeakerTargetPoint()
    distance=pythag(robotPose.x,robotPose.y,targetX,targetY)
    shootSubsys.setTargetDistance(distance)
    return distance


class setShooterDistanceCommand(commands2.Command): #This class is used for estimating where the robot needs to look based on its velocity. 
    def __init__(self,driveSubsys:driveTrainSubsys,shootSubsys:AuxilarySystems.shooterSubsys.shooterSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.shooterSubsys= shootSubsys
        self.dt=driveSubsys

    def execute(self):
        updateShooterDistanceFromDrivePose(self.dt,self.shooterSubsys)

    def end(self,interrupted):
        self.driveSubsys.overideInput()

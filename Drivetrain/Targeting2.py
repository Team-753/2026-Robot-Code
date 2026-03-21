import commands2,wpimath.controller,wpimath.trajectory
from math import atan2,pi,sqrt
import wpilib
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
from wpilib import Timer
import wpilib
import AuxilarySystems.shooterSubsys
from AuxilarySystems import auxiliaryConfig
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


def getTargetPointDistanceMeters(robotPose, targetX, targetY):
    return pythag(robotPose.x, robotPose.y, targetX, targetY)


def getSpeakerDistanceMeters(robotPose):
    targetX, targetY = getSpeakerTargetPoint()
    return getTargetPointDistanceMeters(robotPose, targetX, targetY)


def getTargetRotationRadians(robotPose, targetX, targetY):
    return atan2(robotPose.y-targetY, robotPose.x-targetX)


def getTargetAimErrorRadians(robotPose, targetX, targetY):
    desiredRotation = getTargetRotationRadians(robotPose, targetX, targetY)
    currentRotation = robotPose.rotation().radians()
    return (desiredRotation-currentRotation+pi)%(2*pi)-pi


def getTargetAimErrorDegrees(robotPose, targetX, targetY):
    return abs(getTargetAimErrorRadians(robotPose, targetX, targetY) * 180 / pi)


def isTargetLocked(robotPose, targetX, targetY, toleranceDegrees=None):
    if toleranceDegrees is None:
        toleranceDegrees = auxiliaryConfig.autoTargetAimToleranceDegrees
    return getTargetAimErrorDegrees(robotPose, targetX, targetY) <= toleranceDegrees


def isSpeakerLocked(robotPose, toleranceDegrees=None):
    targetX, targetY = getSpeakerTargetPoint()
    return isTargetLocked(robotPose, targetX, targetY, toleranceDegrees)

class targetPointCommand(commands2.Command): #This class points the robot so the rear shooter faces the target.
    def __init__(self,driveSubsys:driveTrainSubsys,tx=None,ty=None)-> None:
        super().__init__()
        self.tx,self.ty=tx,ty
        self.driveSubsys=driveSubsys
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(45,0.05,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-0.1,0.1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.pidInitialized=False
    def _resetController(self,robotPose):
        self.thetaPid.reset(robotPose.rotation().radians()+pi)
        self.pidInitialized=True
    def initialize(self):
        self._resetController(self.driveSubsys.getPoseState())
    def execute(self):
        robotPose=self.driveSubsys.getPoseState()
        if not self.pidInitialized:
            self._resetController(robotPose)
        if self.tx is None or self.ty is None:
            targetX,targetY=getSpeakerTargetPoint()
        else:
            targetX,targetY=self.tx,self.ty
        desiredRotation=getTargetRotationRadians(robotPose,targetX,targetY)
        output=self.thetaPid.calculate(wpimath.geometry.Rotation2d(robotPose.rotation().radians()+pi).radians(),desiredRotation)
        print(output)
        self.driveSubsys.overideInput(rot=output)
    def end(self,interrupted):
        self.pidInitialized=False
        self.driveSubsys.overideInput()

class targetPointWithLeadCommand(commands2.Command): #This class is used for estimating where the robot needs to look based on its velocity. 
    def __init__(self,driveSubsys:driveTrainSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.TARGET_POINT_BLUE = TARGET_POINT_BLUE
        self.TARGET_POINT_RED = TARGET_POINT_RED
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(45,0.0007,0.15,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.thetaPid.setTolerance(0.6)
        self.robotPose=None
        self.lastPose=None
        self.lastTimestamp=None
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
    
    def getTOF(self):
        pass
        #print(self.lookupTable.loc[self.lookupTable["rpm"].get(1)])
    def initialize(self):
        self.robotPose=self.driveSubsys.getPoseState()
        self.lastPose=self.robotPose
        self.lastTimestamp=Timer.getFPGATimestamp()

    def calucateVelocity(self): #Used to calculate the velocity of the robot so that we can adjust the target point
        self.robotPose=self.driveSubsys.getPoseState()
        currentTime=Timer.getFPGATimestamp()
        if self.lastPose is None or self.lastTimestamp is None:
            self.lastPose=self.robotPose
            self.lastTimestamp=currentTime
            return [0.0,0.0,0.0]

        deltaTime=currentTime-self.lastTimestamp
        if deltaTime <= 1e-4:
            return [0.0,0.0,deltaTime]

        velx=(self.robotPose.x-self.lastPose.x)/deltaTime
        vely=(self.robotPose.y-self.lastPose.y)/deltaTime
        self.lastPose=self.robotPose
        self.lastTimestamp=currentTime
        return [velx,vely,deltaTime]

    def adjustedTargetPoint(self,targetPoint,tof,velocities): #Returns the point we are looking for. 
        tx=targetPoint[0]-(tof*velocities[0])
        ty=targetPoint[1]-(tof*velocities[1])
        return[tx,ty]

    def execute(self):
        velocities=self.calucateVelocity()
        leadTargetPoint=self.adjustedTargetPoint(self._get_target_point(),0.9,velocities)
        desiredRotation=atan2(leadTargetPoint[1]-self.robotPose.y,leadTargetPoint[0]-self.robotPose.x)
        print(leadTargetPoint)
        output=self.thetaPid.calculate(wpimath.geometry.Rotation2d(self.robotPose.rotation().radians()-pi).radians(),desiredRotation)
        self.driveSubsys.overideInput(rot=output)
            
    def end(self,interrupted):
        self.robotPose=None
        self.lastPose=None
        self.lastTimestamp=None
        self.driveSubsys.overideInput()


class targetPointWithShootingCommand(commands2.Command): #This class is used for estimating where the robot needs to look based on its velocity. 
    def __init__(self,driveSubsys:driveTrainSubsys,shootSubsys:AuxilarySystems.shooterSubsys.shooterSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.shooterSubsys= shootSubsys
        self.TARGET_POINT_BLUE = TARGET_POINT_BLUE
        self.TARGET_POINT_RED = TARGET_POINT_RED
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(45,0.0007,0.15,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.thetaPid.setTolerance(0.6)
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
    
    def getTOF(self):
        pass
        #print(self.lookupTable.loc[self.lookupTable["rpm"].get(1)])
    def calucateVelocity(self): #Used to calculate the velocity of the robot so that we can adjust the target point
        try:
            print(pastPosition)
            pastPosition=self.robotPose
            pastTime=self.currentTime
        except:
            print("Error Storing Position to past")
        self.robotPose=self.driveSubsys.getPoseState()
        self.currentTime=Timer.getFPGATimestamp()
        try:
            if pastPosition:
                deltaTime=self.currentTime-pastTime
                velx=self.robotPose.x-pastPosition.x
                vely=self.robotPose.y-pastPosition.y
                velx=velx*(1/deltaTime)
                vely=vely*(1/deltaTime)
        except:
            velx=0
            vely=0
            deltaTime=0
            print("error calculating velocity")
        pastPosition=self.robotPose
        return [velx,vely,deltaTime]

    def adjustedTargetPoint(self,targetPoint,tof,velocities): #Returns the point we are looking for. 
        tx=targetPoint[0]-(tof*velocities[0])
        ty=targetPoint[1]-(tof*velocities[1])
        return[tx,ty]

    def execute(self):
        velocities=self.calucateVelocity()
        leadTargetPoint=self.adjustedTargetPoint(self._get_target_point(),0.9,velocities)
        desiredRotation=atan2(leadTargetPoint[1]-self.robotPose.y,leadTargetPoint[0]-self.robotPose.x)
        print(leadTargetPoint)
        output=self.thetaPid.calculate(wpimath.geometry.Rotation2d(self.robotPose.rotation().radians()-pi).radians(),desiredRotation)
        self.driveSubsys.overideInput(rot=output)
        if self.thetaPid.getPositionError()<=0.17:
            distance=pythag(self.robotPose.x,self.robotPose.y,leadTargetPoint[0],leadTargetPoint[1])
            self.shooterSubsys.setTargetDistance(distance)
            self.shooterSubsys.autoShootStart()
        else:
            self.shooterSubsys.autoShootStop()

    def end(self,interrupted):
        self.driveSubsys.overideInput()

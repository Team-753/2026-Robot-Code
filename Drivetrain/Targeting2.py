import commands2,wpimath.controller,wpimath.trajectory
from math import atan2,pi
import wpilib
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
from wpilib import Timer
import wpilib
        
#NOTICE: This targeting system assumes the metric system. 

#Unfortunantly, Cheeseburgers-per-hour was not easy to implement into the code (plus I am lazy) -Ryan T
class targetPointCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,tx,ty):
        self.tx,self.ty=tx,ty
        self.driveSubsys=driveSubsys
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(65,0.06,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
    def execute(self):
        robotPose=self.driveSubsys.getPoseState()
        desiredRotation=atan2(self.ty-robotPose.y,self.tx-robotPose.x)
        output=self.thetaPid.calculate(robotPose.rotation().radians(),desiredRotation)
        self.driveSubsys.overideInput(rot=output)
    def end(self,interrupted):
        self.driveSubsys.overideInput()
class targetPointWithLeadCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        super().__init__()
        self.driveSubsys = driveSubsys
        self.TARGET_POINT_BLUE = (11.91497, 4.03514)
        self.TARGET_POINT_RED = (4.62507,4.03514)
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(65,0.06,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
    def _get_alliance(self): #We need this information so that we dont score in the wrong hub.
        try:
            alliance = wpilib.DriverStation.getAlliance()
        except Exception:
            return None
        return alliance

    def _get_target_point(self): #Sets the target point based on the team we are on. 
        alliance = self._get_alliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return [self.TARGET_POINT_RED[0],self.TARGET_POINT_RED[1]]
        return [self.TARGET_POINT_BLUE[0],self.TARGET_POINT_BLUE[1]]

    def timestampPose(self):#Used to timestamp where the robot is so that we can run velocity calulations. 
        return (self.driveSubsys.getPoseState,self.currentTime)

    def calucateVelocity(self): #Used to calculate the velocity of the robot so that we can adjust the target point
        try:
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
            print("error calculating velocity")
        pastPosition=self.robotPose
        return [velx,vely,deltaTime]

    def adjustedTargetPoint(self,targetPoint,tof,velocities): #Returns the point we are looking for. 
        tx=targetPoint[0]-(tof*velocities[0])
        ty=targetPoint[1]-(tof*velocities[1])
        return[tx,ty]

    def execute(self):
        print("executing")
        velocities=self.calucateVelocity()
        leadTargetPoint=self.adjustedTargetPoint(self._get_target_point(),0.4,velocities)
        desiredRotation=atan2(leadTargetPoint[1]-self.robotPose.y,leadTargetPoint[0]-self.robotPose.x)
        output=self.thetaPid.calculate(self.robotPose.rotation().radians(),desiredRotation)
        self.driveSubsys.overideInput(rot=output)
        #print(self.timestampPose())
        print("Sending Time Data")
    def end(self,interrupted):
        self.driveSubsys.overideInput()

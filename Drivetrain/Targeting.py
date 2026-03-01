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
        self.TARGET_POINT_RED = (4.62507, 4.03514)
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(65,0.06,0.1,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
        self.thetaPid.setIntegratorRange(-1,1)
        self.thetaPid.enableContinuousInput(-pi,pi)
        self.lastSample = None
        self.velocity = (0.0,0.0,0.0) # vx, vy, omega (field-relative m/s, m/s, rad/s)
        self.projectileFlightTime = 0.0 # seconds
        
    def _get_alliance(self): #We need this information so that we dont score in the wrong hub.
        try:
            alliance = wpilib.DriverStation.getAlliance()
        except Exception:
            return None
        return alliance

    def _get_target_point(self): #Sets the target point based on the team we are on. 
        alliance = self._get_alliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            return self.TARGET_POINT_RED
        return self.TARGET_POINT_BLUE

    def _poseXY(self, pose):
        if hasattr(pose, "X") and callable(getattr(pose, "X")):
            return float(pose.X()), float(pose.Y())
        return float(pose.x), float(pose.y)

    def _angleDelta(self,current,previous):
        return (current - previous + pi) % (2*pi) - pi

    def timestampPose(self):#Used to timestamp where the robot is so that we can run velocity calulations.
        return (Timer.getFPGATimestamp(), self.driveSubsys.getPoseState())

    def calucateVelocity(self): #Used to calculate the velocity of the robot so that we can adjust the target point
        sampleTime, samplePose = self.timestampPose()

        if self.lastSample is None:
            self.lastSample = (sampleTime, samplePose)
            return self.velocity

        lastTime, lastPose = self.lastSample
        dt = sampleTime - lastTime
        if dt <= 1e-4:
            return self.velocity

        xNow, yNow = self._poseXY(samplePose)
        xOld, yOld = self._poseXY(lastPose)
        thetaNow = samplePose.rotation().radians()
        thetaOld = lastPose.rotation().radians()

        vx = (xNow - xOld) / dt
        vy = (yNow - yOld) / dt
        omega = self._angleDelta(thetaNow, thetaOld) / dt

        self.velocity = (vx, vy, omega)
        self.lastSample = (sampleTime, samplePose)
        return self.velocity

    def setProjectileFlightTime(self, flightTimeSeconds):
        self.projectileFlightTime = max(0.0, float(flightTimeSeconds))

    def adjustedTargetPoint(self): #Returns the point we are looking for. 
        targetX, targetY = self._get_target_point()
        vx, vy, _ = self.velocity
        leadX = targetX - (vx * self.projectileFlightTime)
        leadY = targetY - (vy * self.projectileFlightTime)
        return leadX, leadY

    def execute(self):
        robotPose=self.driveSubsys.getPoseState()
        self.calucateVelocity()
        targetX, targetY = self.adjustedTargetPoint()
        robotX, robotY = self._poseXY(robotPose)
        desiredRotation=atan2(targetY-robotY,targetX-robotX)
        output=self.thetaPid.calculate(robotPose.rotation().radians(),desiredRotation)
        #print(robotPose.rotation().radians(),desiredRotation,self.ty-robotPose.y)
        self.driveSubsys.overideInput(rot=output)
    def end(self,interrupted):
        self.driveSubsys.overideInput()

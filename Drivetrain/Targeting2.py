import commands2,wpimath.controller
from math import atan2,pi
from Drivetrain.swerveSubsys import overideRobotInput,driveTrainSubsys
class targetPointCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,tx,ty):
        self.tx,self.ty=tx,ty
        self.driveSubsys=driveSubsys
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(2*pi,2*pi))
    def execute(self):
        robotPose=self.driveSubsys.getPoseState()
        desiredRotation=atan2(robotPose.y-self.ty,robotPose.x-self.tx)
        output=self.thetaPid.calculate(robotPose.rotation().radians()-desiredRotation)
        self.driveSubsys.overideInput(rot=output)
    def end(self,interrupted):
        self.driveSubsys.overideInput()
        
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
        print(robotPose.rotation().radians(),desiredRotation,self.ty-robotPose.y)
        self.driveSubsys.overideInput(rot=output)
    def end(self,interrupted):
        self.driveSubsys.overideInput()

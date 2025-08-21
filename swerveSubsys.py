import commands2,phoenix6,wpimath,navx
from math import pi
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
from wpilib import AnalogEncoder
import wpimath.trajectory
from robotConfig import swerveStuff
class swerveSubsys():
    def __init__(self,driveID,turnID,turnSensorID=None):
        super().__init__()
        self.driveMotor=phoenix6.hardware.TalonFX(driveID)
        self.turnMotor=phoenix6.hardware.TalonFX(turnID)
        self.turnVariable=self.turnMotor.get_position()
        #DRIVE CONFIG
        driveConfig=phoenix6.configs.TalonFXConfiguration()
        driveConfig.slot0.k_p = 0.3
        driveConfig.slot0.k_i = 0.0
        driveConfig.slot0.k_d= 0
        driveConfig.slot0.k_v = 0.01
        driveConfig.slot0.k_s = 0.0875
        driveConfig.feedback.sensor_to_mechanism_ratio = 8.14
        #TURN CONFIG
        turnConfig=phoenix6.configs.TalonFXConfiguration()
        turnConfig.slot1.k_p = 50
        turnConfig.slot1.k_i = 0
        turnConfig.slot1.k_d= 0
        turnConfig.slot1.k_v = 0.01
        turnConfig.feedback.sensor_to_mechanism_ratio = 12.8
        turnConfig.closed_loop_general.continuous_wrap = True
        self.driveMotor.configurator.apply(driveConfig)
        self.turnMotor.configurator.apply(turnConfig)
        self.postion=phoenix6.controls.PositionVoltage(0).with_slot(1)
        self.dutyCycle=phoenix6.controls.VelocityVoltage(0).with_slot(0)
        if turnSensorID!=None:
            self.hasSensor=True
            self.turnSensor=AnalogEncoder(turnSensorID)
            self.turnVariable=self.turnSensor.get()
        self.turnMotor.set_position(self.turnSensor.get()-swerveStuff.offsetList[turnSensorID])
    def setState(self,desRot,desSpeed):
        self.turnMotor.set_control(self.postion.with_position(desRot))
        self.driveMotor.set_control(self.dutyCycle.with_velocity(desSpeed*8.14))
    def getRot(self):
        return self.turnMotor.get_position().value
    def getState(self):
        return wpimath.kinematics.SwerveModulePosition(self.driveMotor.get_position().value*0.095*pi,wpimath.geometry.Rotation2d(self.turnSensor.get()))
    def reZero(self,id):
        self.turnMotor.set_position(self.turnSensor.get()-swerveStuff.offsetList[id])
class driveTrainSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        for i in range(4):
            num=(i+1)*2
            string=str("self.swerve"+str(i)+"=swerveSubsys("+str(num-1)+","+str(num)+","+str(i)+")")
            print(string)
            exec(string)
        self.navX=navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.swerveKinematics=wpimath.kinematics.SwerveDrive4Kinematics(wpimath.geometry.Translation2d(0.26,0.32),wpimath.geometry.Translation2d(0.26,-0.32),wpimath.geometry.Translation2d(-0.26,-0.32),wpimath.geometry.Translation2d(-0.26,0.32))

        #DOES NOT USE GYRO DATA, REPLACE WITH ESTIMATOR
        self.odometry=wpimath.kinematics.SwerveDrive4Odometry(self.swerveKinematics,self.navX.getRotation2d(),self.getSwerveState(),wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(0,0),wpimath.geometry.Rotation2d(0)))
    def setState(self,fb,lr,rot):
        #swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds(fb,lr,rot))
        #print(self.swerve0.tempFunc(),self.swerve1.tempFunc(),self.swerve2.tempFunc(),self.swerve3.tempFunc())
        self.swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(fb,lr,rot,self.navX.getRotation2d()))
        for i in range(4):
            exec(str("self.swerveNumbers["+str(i)+"].optimize(wpimath.geometry.Rotation2d(self.swerve"+str(i)+".getRot()*2*pi))"))
            exec(str("self.swerve"+str(i)+".setState(self.swerveNumbers["+str(i)+"].angle.degrees()/360,self.swerveNumbers["+str(i)+"].speed_fps/3.18)"))
    def getPoseState(self):
        return self.odometry.getPose()
    def periodic(self):
        self.odometry.update(self.navX.getRotation2d(),self.getSwerveState())
        return super().periodic()
    def getSwerveState(self):
        string=[]
        for i in range(4):
            exec("string.append(self.swerve"+str(i)+".getState())")
        return string
class joystickSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandXboxController):
        self.myJoy=joystick
        super().__init__()
    def getY(self):
        return self.myJoy.getLeftY()
    def getX(self):
        return self.myJoy.getLeftX()
    def getZ(self):
        return self.myJoy.getRightX()
class hotasSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandJoystick):
        self.myJoy=joystick
        super().__init__()
    def getX(self):
        return self.myJoy.getRawAxis(axis=0)
    def getZ(self):
        return self.myJoy.getRawAxis(axis=5)
    def getY(self):
        return self.myJoy.getRawAxis(axis=1)
class driveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:joystickSubsys):
        super().__init__()
        self.addRequirements(driveSubsys,joySubsys)
        self.driveTrain,self.joystick=driveSubsys,joySubsys
    def execute(self):
        #print(self.joystick.getX(),self.joystick.getY(),self.joystick.getZ())
        print(self.driveTrain.getPoseState())
        self.driveTrain.setState(self.joystick.getY(),self.joystick.getX(),self.joystick.getZ())#self.joystick.getZ()*2)
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        super().__init__()
        config = wpimath.trajectory.TrajectoryConfig.fromFps(12, 12)
        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(0,5,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(0.1,0,0),cont.PIDController(0.1,0,0),cont.ProfiledPIDControllerRadians(0.1,0,0))
        self.trajectory=wpimath.trajectory.TrajectoryGenerator.generateTrajectory(startPos,end=endPos,config=config)
    def execute(self,desCoord):
        return self.holoCont.calculate(self.driveSubsys.getPoseState(),self.trajectory,)
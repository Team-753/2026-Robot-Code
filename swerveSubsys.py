import commands2,phoenix6,wpimath,navx
from math import pi
import math
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.estimator
from wpilib import AnalogEncoder
import wpimath.trajectory
import swerveConfig
class swerveSubsys():
    def __init__(self,driveID,turnID,turnSensorID=None):
        super().__init__()
        self.driveMotor=phoenix6.hardware.TalonFX(driveID)
        self.turnMotor=phoenix6.hardware.TalonFX(turnID)
        self.turnVariable=self.turnMotor.get_position()
        turnSensorType=swerveConfig.swerveEncoderType

        #DRIVE CONFIG
        driveConfig=phoenix6.configs.TalonFXConfiguration()
        driveConfig.slot0.k_p = 2
        driveConfig.slot0.k_i = 0.0
        driveConfig.slot0.k_d= 0
        driveConfig.slot0.k_v = 0.01
        driveConfig.slot0.k_s = 0.0875
        driveConfig.feedback.sensor_to_mechanism_ratio = swerveConfig.swerveDriveRatio

        #TURN CONFIG
        turnConfig=phoenix6.configs.TalonFXConfiguration()
        turnConfig.slot1.k_p = 20
        turnConfig.slot1.k_i = 0
        turnConfig.slot1.k_d= 0
        turnConfig.slot1.k_v = 0.01
        turnConfig.slot1.k_s = 0.0875
        #turnConfig.feedback.sensor_to_mechanism_ratio = swerveConfig.swerveTurnRatio
        turnConfig.closed_loop_general.continuous_wrap = True

        #CONTROL MODES
        self.postion=phoenix6.controls.PositionVoltage(0).with_slot(1)
        self.dutyCycle=phoenix6.controls.VelocityVoltage(0).with_slot(0)

        #CHECKS IF USING TURN SENSOR
        if turnSensorID!=None:
            self.hasSensor=True
            self.hasWpiEnc=False
            if turnSensorType=="wpilibEncoder":
                self.hasWpiEnc=True
                self.turnSensor=AnalogEncoder(turnSensorID)
                self.turnVariable=self.turnSensor.get()
            elif turnSensorType=="canCoder":
                #CAN CODER CONFIG

                #ADDITIONAL TURN CONFIG
                self.turnSensor=phoenix6.hardware.CANcoder(turnSensorID)
                turnConfig.feedback.feedback_remote_sensor_id=turnSensorID
                turnConfig.feedback.feedback_sensor_source=phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER        
        #APPLY CONFIGS
        self.driveMotor.configurator.apply(driveConfig)
        self.turnMotor.configurator.apply(turnConfig)
        if self.hasWpiEnc:
            print("HAS WPIENC")
            self.turnMotor.set_position(self.turnSensor.get()-swerveConfig.offsetList[turnSensorID])
    def setState(self,desRot,desSpeed):
        self.turnMotor.set_control(self.postion.with_position(desRot))
        self.driveMotor.set_control(self.dutyCycle.with_velocity(desSpeed))
    def getRot(self):
        if self.hasWpiEnc:
            return self.turnMotor.get_position().value
        else:
            return self.turnSensor.get_absolute_position().value
    def getState(self):
        return wpimath.kinematics.SwerveModulePosition(self.driveMotor.get_position().value*0.095*pi,wpimath.geometry.Rotation2d(self.turnMotor.get_position().value_as_double*pi*2))
    def reZero(self,id):
        self.turnMotor.set_position(self.turnSensor.get()-swerveConfig.offsetList[id])
class driveTrainSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        for i in range(4):
            string=str("self.swerve"+str(i)+"=swerveSubsys("+str(swerveConfig.swerveDriveIds[i])+","+str(swerveConfig.swerveTurnIds[i])+","+str(swerveConfig.swerveEncoderIds[i])+")")
            exec(string)
        self.navX=navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        widthN=swerveConfig.swerveBaseWidth/2
        lengthN=swerveConfig.swerveBaseLength/2
        self.swerveKinematics=wpimath.kinematics.SwerveDrive4Kinematics(wpimath.geometry.Translation2d(widthN/2,lengthN/2),wpimath.geometry.Translation2d(widthN/2,-lengthN/2),wpimath.geometry.Translation2d(-widthN/2,-lengthN/2),wpimath.geometry.Translation2d(-widthN/2,lengthN/2))

        #DOES NOT USE GYRO DATA, REPLACE WITH ESTIMATOR
        #self.poseEstimator=wpimath.estimator.SwerveDrive4PoseEstimator(self.swerveKinematics,wpimath.geometry.Rotation2d.fromDegrees(self.navX.getRotation2d().degrees),self.getSwerveState(),wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(0,0),wpimath.geometry.Rotation2d(0)))
        self.odometry=wpimath.kinematics.SwerveDrive4Odometry(self.swerveKinematics,wpimath.geometry.Rotation2d.fromDegrees(self.navX.getRotation2d().degrees()),self.getSwerveState(),wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(0,0),wpimath.geometry.Rotation2d(0)))
    def setState(self,fb,lr,rot):       
        #self.swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds(fb,lr,rot))
        #print(self.swerve0.getRot(),self.swerve1.getRot(),self.swerve2.getRot(),self.swerve3.getRot())
        self.swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(fb,lr,rot,wpimath.geometry.Rotation2d.fromDegrees(-self.navX.getRotation2d().degrees())))
        #print(self.swerveNumbers[0].angle.degrees(),self.swerveNumbers[0].speed_fps)
        for i in range(4):
            exec(str("self.swerveNumbers["+str(i)+"].optimize(wpimath.geometry.Rotation2d(self.swerve"+str(i)+".getRot()*2*pi))"))
            exec(str("self.swerve"+str(i)+".setState(self.swerveNumbers["+str(i)+"].angle.degrees()/360,self.swerveNumbers["+str(i)+"].speed_fps)"))
    def getPoseState(self):
        odo=self.odometry.getPose()
        #print(odo)
        return odo
        return wpimath.geometry.Pose2d(wpimath.geometry.Translation2d(-odo.x,-odo.y),odo.rotation())
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
        return self.myJoy.getRawAxis(axis=2)
    def getY(self):
        return self.myJoy.getRawAxis(axis=1)
class driveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:joystickSubsys):
        super().__init__()
        self.addRequirements(driveSubsys,joySubsys)
        self.driveTrain,self.joystick=driveSubsys,joySubsys
    def execute(self):
        #print(self.joystick.getX(),self.joystick.getY(),self.joystick.getZ())
              #,self.driveTrain.getSwerveState())
        print(self.driveTrain.getPoseState())
        self.driveTrain.setState(-self.joystick.getY()*2,self.joystick.getX()*2,-self.joystick.getZ()*4)#self.joystick.getZ()*2)
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        super().__init__()
        config = wpimath.trajectory.TrajectoryConfig.fromFps(12, 12)
        config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,1,wpigeo.Rotation2d.fromDegrees(180))
        endPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(180))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(0.3,0,0),cont.PIDController(0.3,0,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.trajectory=wpimath.trajectory.TrajectoryGenerator.generateTrajectory([startPos,endPos],config=config)
    def execute(self):
        self.goal=self.trajectory.sample(1.49)
        speeds=self.holoCont.calculate(self.driveSubsys.getPoseState(),self.goal,wpimath.geometry.Rotation2d(0))
        print(self.driveSubsys.getPoseState(),self.driveSubsys.getPoseState().y_feet)
        self.driveSubsys.setState(speeds.vx,speeds.vy,speeds.omega)
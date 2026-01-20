import commands2,phoenix6,wpimath,wpilib
from math import pi
import math
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator

from limelight import LimelightCamera

from wpimath import estimator

from wpilib import AnalogEncoder,Timer, Field2d
import wpimath.trajectory
import swerveConfig
from customFunctions import curveControl
class swerveSubsys():
    def __init__(self,driveID,turnID,turnSensorID=None):


        super().__init__()
        self.driveMotor=phoenix6.hardware.TalonFX(driveID)
        self.turnMotor=phoenix6.hardware.TalonFX(turnID)
        turnSensorType=swerveConfig.swerveEncoderType

        

        #DRIVE CONFIG
        driveConfig=phoenix6.configs.TalonFXConfiguration()
        driveConfig.slot0.k_p = 1
        driveConfig.slot0.k_i = 0.0
        driveConfig.slot0.k_d= 0
        driveConfig.slot0.k_v = 0.01
        driveConfig.slot0.k_s = 0.0875
        driveConfig.feedback.sensor_to_mechanism_ratio = swerveConfig.swerveDriveRatio

        #TURN CONFIG
        turnConfig=phoenix6.configs.TalonFXConfiguration()
        turnConfig.slot1.k_p = 50
        turnConfig.slot1.k_i = 0.0001
        turnConfig.slot1.k_d= 0.1
        turnConfig.slot1.k_v = 0.01
        turnConfig.slot1.k_s = 0.0875
        turnConfig.feedback.sensor_to_mechanism_ratio = swerveConfig.swerveTurnRatio
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
                self.turnSensor=phoenix6.hardware.CANcoder(turnSensorID)
                turnSensorConfig=phoenix6.configs.CANcoderConfiguration()
                turnSensorConfig.magnet_sensor.magnet_offset=swerveConfig.offsetList[swerveConfig.swerveEncoderIds.index(turnSensorID)]
                turnSensorConfig.magnet_sensor.absolute_sensor_discontinuity_point=0.5 #FACTORY
                turnSensorConfig.magnet_sensor.sensor_direction=phoenix6.signals.SensorDirectionValue.CLOCKWISE_POSITIVE
                self.turnSensor.configurator.apply(turnSensorConfig)
                #ADDITIONAL TURN CONFIG
                turnConfig.feedback.feedback_sensor_source=phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER
                turnConfig.feedback.feedback_remote_sensor_id=turnSensorID 
                       
        #APPLY CONFIGS
        self.driveMotor.configurator.apply(driveConfig)
        self.turnMotor.configurator.apply(turnConfig)
        if self.hasWpiEnc:
            print("HAS WPIENC")
            self.turnMotor.set_position(self.turnSensor.get()-swerveConfig.offsetList[swerveConfig.swerveEncoderIds.index(turnSensorID)])

            
    def setState(self,desRot,desSpeed):
        self.turnMotor.set_control(self.postion.with_position(desRot))
        self.driveMotor.set_control(self.dutyCycle.with_velocity(desSpeed))

    def getRot(self):
        if self.hasWpiEnc:
            return self.turnMotor.get_position().value
        else:
            return self.turnSensor.get_absolute_position().value

    def getState(self):
        return wpimath.kinematics.SwerveModulePosition(self.driveMotor.get_position().value*0.095*pi,wpimath.geometry.Rotation2d.fromRotations(-self.turnMotor.get_position().value))

    def debug(self):
        return self.turnSensor.get_position().value

class driveTrainSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        #self.swerveSubsystems = []
        for i in range(4):
            #self.swerveSubsystems.append(swerveSubsys())
            string=str("self.swerve"+str(i)+"=swerveSubsys("+str(swerveConfig.swerveDriveIds[i])+","+str(swerveConfig.swerveTurnIds[i])+","+str(swerveConfig.swerveEncoderIds[i])+")")
            exec(string)
        if swerveConfig.robotCompassType=="pidgeon":
            self.compass=phoenix6.hardware.Pigeon2(swerveConfig.robotCompassId)
            self.compass.reset()
            self.robotRotation=self.compass.getRotation2d()
    

        #Vision (Ryan)
        self.limeLight = LimelightCamera(swerveConfig.cameraName)

        #Pose Setup for Estimator (Ryan)
        self.field = Field2d() #creates a field on shuffleboard, useful for debugging autos

        widthN = swerveConfig.swerveBaseWidth/2
        lengthN = swerveConfig.swerveBaseLength/2

        self.swerveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(wpimath.geometry.Translation2d(widthN/2,lengthN/2),wpimath.geometry.Translation2d(widthN/2,-lengthN/2),wpimath.geometry.Translation2d(-widthN/2,-lengthN/2),wpimath.geometry.Translation2d(-widthN/2,lengthN/2))

        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(
            self.swerveKinematics,
            self.robotRotation,
            self.getSwerveState(),
            wpimath.geometry.Pose2d(
                wpimath.geometry.Translation2d(5, 0),
                wpimath.geometry.Rotation2d(0),
            ),
            swerveConfig.wheelDistrustLevel,
            swerveConfig.visionDistrustLevel,
        )
    
    def setState(self,fb,lr,rot):       
        
        self.swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(fb,lr,rot,-self.compass.getRotation2d()))#FIELD ALIGN
        
        for i in range(4):
            #IF JITTERING WITH CORRECT PID, REVERSE OPTIMIZE ANGLE INPUT
            exec(str("self.swerveNumbers["+str(i)+"].optimize(wpimath.geometry.Rotation2d.fromRotations(self.swerve"+str(i)+".getRot()))"))
            exec(str("self.swerve"+str(i)+".setState(self.swerveNumbers["+str(i)+"].angle.degrees()/360,self.swerveNumbers["+str(i)+"].speed_fps)"))
    def getPoseState(self):
        odo=self.odometry.getPose()
        return odo

    def periodic(self):

        time = Timer.getFPGATimestamp()

        if  self.limeLight.hasDetection() == True:
            #check to see if the robot can see an april tag

            posedata,latency = self.limeLight.getPoseData()

            lockTime = time - (latency/1000) #Take the locktime minus the latency (in miliseconds) to know how long in the past locking was
            self.poseEstimator.addVisionMeasurement(posedata,lockTime)
        currentPose = self.poseEstimator.update(self.compass.getRotation2d(), self.getSwerveState())
        #update the pose estimator with our most up to date info on where the robot is from all the systems


        self.field.setRobotPose(currentPose) #update the position of the robot on the field in shuffleboard for debugging

    #    return super().periodic()
    def getSwerveState(self):
        string=[]
        for i in range(4):
            exec("string.append(self.swerve"+str(i)+".getState())")
        return string

##DIFFERENT INPUT DEVICE CONFIGS
class XboxControllerSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandXboxController):
        self.myJoy=joystick
        super().__init__()
    def getY(self):
        return self.myJoy.getLeftY()
    def getX(self):
        return self.myJoy.getLeftX()
    def getZ(self):
        return self.myJoy.getRightX()

class JoystickSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandJoystick):
        self.myJoy=joystick
        super().__init__()
    def getX(self):
        return self.myJoy.getRawAxis(axis=0)
    def getZ(self):
        return self.myJoy.getRawAxis(axis=2)
    def getY(self):
        return self.myJoy.getRawAxis(axis=1)

class VKBJoystickSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandJoystick):
        self.myJoy=joystick
        super().__init__()
    def getX(self):
        return self.myJoy.getRawAxis(axis=0)
    def getZ(self):
        return self.myJoy.getRawAxis(axis=5)
    def getY(self):
        return self.myJoy.getRawAxis(axis=1)

##SENDING COMMANDS TO DRIVETRAIN
##NOTE APPLY EXPONENTIAL TO VECTOR INTEAD OF JOYSTICK COMPONETS,NOTE
class driveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:globals()[swerveConfig.driveController+"Subsys"]):
       super().__init__()
       self.addRequirements(driveSubsys,joySubsys)
       self.driveTrain,self.joystick=driveSubsys,joySubsys

    def execute(self):
        self.driveTrain.setState(curveControl(self.joystick.getY(),2.5)*swerveConfig.driveSpeed,-curveControl(self.joystick.getX(),2.5)*swerveConfig.driveSpeed,curveControl(-self.joystick.getZ(),2)*swerveConfig.driveTurnSpeed)
        print(self.driveTrain.odometry.getPose(),self.driveTrain.compass.getRotation2d())

class fieldOrientReorient(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:globals()[swerveConfig.driveController+"Subsys"]):
        self.addRequirements(driveSubsys,joySubsys)

class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.addRequirements(driveSubsys)
        self.driveSubsys=driveSubsys
        cont=wpimath.controller
        wpigeo=wpimath.geometry
        super().__init__()
        aconfig = wpimath.trajectory.TrajectoryConfig(1,1)

        #config.setReversed(True)
        #IMPORTANT STUFF
        startPos=wpigeo.Pose2d.fromFeet(0,0,wpigeo.Rotation2d.fromDegrees(0))
        endPos=wpigeo.Pose2d.fromFeet(-3,0,wpigeo.Rotation2d.fromDegrees(0))
        self.holoCont=cont.HolonomicDriveController(cont.PIDController(1,0,0),cont.PIDController(1,0,0),cont.ProfiledPIDControllerRadians(0.3,0,0,wpimath.trajectory.TrapezoidProfileRadians.Constraints(pi,pi)))
        self.trajectory=wpimath.trajectory.TrajectoryGenerator.generateTrajectory([startPos,endPos],config=config)
    def execute(self):
        self.goal=self.trajectory.sample(self.clock.get())
        speeds=self.holoCont.calculate(self.driveSubsys.getPoseState(),self.goal,wpimath.geometry.Rotation2d(0))
        #print(self.driveSubsys.getPoseState())#,self.driveSubsys.getPoseState().y_feet)
        print(speeds.vx,speeds.vy,speeds.omega,self.clock.get())
        self.driveSubsys.setState(speeds.vx,speeds.vy,speeds.omega)

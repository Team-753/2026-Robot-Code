import commands2,phoenix6,wpimath,wpilib
from math import pi
import math
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator

from Drivetrain.limelight import LimelightCamera

from wpimath import estimator

from wpilib import AnalogEncoder,Timer, Field2d
import wpimath.trajectory
import Drivetrain.swerveConfig as swerveConfig
from customFunctions import curveControl,vectorCurve
import navx
class swerveSubsys():
    def __init__(self,driveID,turnID,turnSensorID=None,swerveCanivoreName=None):
        if not swerveCanivoreName:
            swerveCanivoreName="rio"
        super().__init__()
        self.driveMotor=phoenix6.hardware.TalonFX(driveID,swerveCanivoreName)
        self.turnMotor=phoenix6.hardware.TalonFX(turnID,swerveCanivoreName)
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
                self.turnSensor=phoenix6.hardware.CANcoder(turnSensorID,swerveCanivoreName)
                turnSensorConfig=phoenix6.configs.CANcoderConfiguration()
                if swerveConfig.debugOffsets:
                    turnSensorConfig.magnet_sensor.magnet_offset=0
                else:
                    turnSensorConfig.magnet_sensor.magnet_offset=1.5-swerveConfig.offsetList[swerveConfig.swerveEncoderIds.index(turnSensorID)]-1
                turnSensorConfig.magnet_sensor.absolute_sensor_discontinuity_point=0.5 #FACTORY
                turnSensorConfig.magnet_sensor.sensor_direction=phoenix6.signals.SensorDirectionValue.CLOCKWISE_POSITIVE
                self.turnSensor.configurator.apply(turnSensorConfig)
                #ADDITIONAL TURN CONFIG
                turnConfig.feedback.feedback_sensor_source=phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER
                turnConfig.feedback.feedback_remote_sensor_id=turnSensorID 
                       
        #APPLY CONFIGS
        self.driveMotor.configurator.apply(driveConfig,0.5)
        self.turnMotor.configurator.apply(turnConfig,0.5)
        if self.hasWpiEnc:
            print("HAS WPIENC")
            self.turnMotor.set_position(self.turnSensor.get()-swerveConfig.offsetList[swerveConfig.swerveEncoderIds.index(turnSensorID)])

            
    def setState(self,desRot,desSpeed):
        if not swerveConfig.debugOffsets:
            self.turnMotor.set_control(self.postion.with_position(desRot))
        self.driveMotor.set_control(self.dutyCycle.with_velocity(desSpeed))

    def getTemperature(self): 
        turnMotorTemperature = self.turnMotor.get_device_temp().refresh()
        driveMotorTemperature = self.driveMotor.get_device_temp().refresh()
        return turnMotorTemperature, driveMotorTemperature

    def getRot(self):
        if self.hasWpiEnc:
            return self.turnMotor.get_position().value
        else:
            return self.turnSensor.get_absolute_position().value

    def getState(self):
        return wpimath.kinematics.SwerveModulePosition(-self.driveMotor.get_position().value*swerveConfig.swerveWheelDiameter*pi,wpimath.geometry.Rotation2d.fromRotations(-self.turnMotor.get_position().value))

    def debug(self):
        return self.turnSensor.get_position().value

class driveTrainSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.overidedInputs=[None,None,None]
        self.resetCompass=False
        self.swerveModules = []
        for i in range(4):
            self.swerveModules.append(swerveSubsys(swerveConfig.swerveDriveIds[i],swerveConfig.swerveTurnIds[i],swerveConfig.swerveEncoderIds[i],swerveConfig.swerveCanivoreName))
        if swerveConfig.robotCompassType=="pidgeon":
            if not swerveConfig.swerveCanivoreName:
                swerveConfig.swerveCanivoreName="rio"
            self.compass=phoenix6.hardware.Pigeon2(swerveConfig.robotCompassId,swerveConfig.swerveCanivoreName)
            self.compass.reset()
            self.robotRotation=self.compass.getRotation2d()
        if swerveConfig.robotCompassType=="navx":
            self.compass=navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
            self.compass.reset()
            self.robotRotation=self.compass.getRotation2d()


        #Vision (Ryan)
        self.limeLight = LimelightCamera(swerveConfig.cameraName)

        #Pose Setup for Estimator (Ryan)
        self.field = Field2d() #creates a field on shuffleboard, useful for debugging autos
        wpilib.SmartDashboard.putData("Field", self.field)

        widthN = swerveConfig.swerveBaseWidth/2
        lengthN = swerveConfig.swerveBaseLength/2

        self.swerveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(-lengthN/2,widthN/2),
            wpimath.geometry.Translation2d(-lengthN/2,-widthN/2),
            wpimath.geometry.Translation2d(lengthN/2,-widthN/2),
            wpimath.geometry.Translation2d(lengthN/2,widthN/2))

        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(
            self.swerveKinematics,
            self.robotRotation,
            self.getSwerveState(),
            wpimath.geometry.Pose2d(
                wpimath.geometry.Translation2d(swerveConfig.startPoseX, swerveConfig.startPoseY),
                wpimath.geometry.Rotation2d.fromDegrees(swerveConfig.startPoseDeg),
            ),
            swerveConfig.wheelDistrustLevel,
            swerveConfig.visionDistrustLevel,
        )
    
    def setState(self,fb,lr,rot):
        inputs=[fb,lr,rot]
        for i in range(3):
            if self.overidedInputs[i]!=None:
                inputs[i]=self.overidedInputs[i]
        if self.resetCompass:
            self.compass.reset()
        
        self.swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(inputs[0],inputs[1],inputs[2],-self.compass.getRotation2d()))#FIELD ALIGN
        if swerveConfig.debugOffsets:
            print(self.swerveModules[0].getRot(),self.swerveModules[1].getRot(),self.swerveModules[2].getRot(),self.swerveModules[3].getRot())
        for i in range(4):
            #IF JITTERING WITH CORRECT PID, REVERSE OPTIMIZE ANGLE INPUT
            self.swerveNumbers[i].optimize(wpimath.geometry.Rotation2d.fromRotations(self.swerveModules[i].getRot()))
            self.swerveModules[i].setState(self.swerveNumbers[i].angle.radians()/(2*pi),self.swerveNumbers[i].speed)
    def getPoseState(self):
        return self.poseEstimator.getEstimatedPosition()

    def periodic(self):

        time = Timer.getFPGATimestamp()

        if self.limeLight.hasDetection():
            print("Beans detected")
            posedata, latency = self.limeLight.getPoseData()
            if posedata is not None and latency is not None:
                lockTime = time - (latency/1000) #Take the locktime minus the latency (in miliseconds) to know how long in the past locking was
                print("adding measurment")
                self.poseEstimator.addVisionMeasurement(posedata,lockTime)
        currentPose = self.poseEstimator.update(self.compass.getRotation2d(), self.getSwerveState())
        #update the pose estimator with our most up to date info on where the robot is from all the systems


        for i, modules in enumerate(self.swerveModules): #TODO add the extra bits
            pass
        


        self.field.setRobotPose(currentPose) #update the position of the robot on the field in shuffleboard for debugging
        wpilib.SmartDashboard.putNumber("Pose X", currentPose.x_feet)
        wpilib.SmartDashboard.putNumber("Pose Y", currentPose.y_feet)
        wpilib.SmartDashboard.putNumber("Pose Deg", currentPose.rotation().degrees())
        wpilib.SmartDashboard.putNumber("Gyro degrees", self.compass.getRotation2d().degrees())

    #    return super().periodic()
    def getSwerveState(self):
        string=[]
        for i in range(4):
            #string.append(self.swerve"+str(i)+".getState())")
            string.append(self.swerveModules[i].getState())
        return string
    def robotRecenter(self,bool):
        self.resetCompass=bool

    def overideInput(self,x=None,y=None,rot=None):
        self.overidedInputs[0]=x
        self.overidedInputs[1]=y
        self.overidedInputs[2]=rot
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
        return wpimath.applyDeadband(self.myJoy.getRawAxis(axis=0),0.05)
    def getZ(self):
        return wpimath.applyDeadband(self.myJoy.getRawAxis(axis=2),0.05)
    def getY(self):
        return wpimath.applyDeadband(self.myJoy.getRawAxis(axis=1),0.05)

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
        curvedDriveValues=vectorCurve(self.joystick.getX(),-self.joystick.getY(),2.5,swerveConfig.driveSpeed)
        self.driveTrain.setState(curvedDriveValues[0],curvedDriveValues[1],curveControl(-self.joystick.getZ(),2)*swerveConfig.driveTurnSpeed)
        pass

class fieldOrientReorient(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys):
        self.dt=driveSubsys
    def execute(self):
        self.dt.robotRecenter(True)
    def end(self, interrupted):
        self.dt.robotRecenter(False)

class overideRobotInput(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,x=None,y=None,theta=None):
        self.dt=driveSubsys
        self.inputs=[x,y,theta]
    def execute(self):
        self.dt.overideInput(self.inputs[0],self.inputs[1],self.inputs[2])
    def end(self, interrupted):
        self.dt.overideInput(None,None,None)

class pointToVelocityVectorCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:globals()[swerveConfig.driveController+"Subsys"]):
        self.dt=driveSubsys
        self.joystick=joySubsys
        self.thetaPid=wpimath.controller.ProfiledPIDControllerRadians(63,0.03,0.05,wpimath.trajectory.TrapezoidProfileRadians.Constraints(4*pi,4*pi))
        self.thetaPid.setIntegratorRange(-1,1)
    def execute(self):
        robotPose=self.dt.getPoseState()
        desiredRotation=math.atan2(self.joystick.getY(),self.joystick.getX())
        output=self.thetaPid.calculate(robotPose.rotation().radians(),desiredRotation)
        self.dt.overideInput(rot=output)
    def end(self,interrupted):
        self.dt.overideInput()

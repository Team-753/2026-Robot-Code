import commands2,phoenix6,wpimath,navx
from math import pi
import wpimath.geometry
import wpimath.kinematics
from wpilib import AnalogEncoder
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
    def tempFunc(self):
        return self.turnSensor.get()
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
    def setState(self,fb,lr,rot):
        #swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds(fb,lr,rot))
        #print(self.swerve0.tempFunc(),self.swerve1.tempFunc(),self.swerve2.tempFunc(),self.swerve3.tempFunc())
        swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(fb,lr,rot,self.navX.getRotation2d()))
        for i in range(4):
            exec(str("swerveNumbers["+str(i)+"].optimize(wpimath.geometry.Rotation2d(self.swerve"+str(i)+".getRot()*2*pi))"))
            exec(str("self.swerve"+str(i)+".setState(swerveNumbers["+str(i)+"].angle.degrees()/360,swerveNumbers["+str(i)+"].speed_fps/3.18)"))
    def setFieldOrientState(self,fb,lr,rot):
        swerveNumbers=self.swerveKinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(lr,fb,rot,self.navX.getRotation2d()))
    def getState(self):
        self.swerve0.getState()
class joystickSubsys(commands2.Subsystem):
    def __init__(self,joystick=commands2.button.CommandXboxController):
        self.myJoy=joystick
        super().__init__()
    def getLY(self):
        return self.myJoy.getLeftY()
    def getLX(self):
        return self.myJoy.getLeftX()
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
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:hotasSubsys):
        super().__init__()
        self.addRequirements(driveSubsys,joySubsys)
        self.driveTrain,self.joystick=driveSubsys,joySubsys
    def execute(self):
        #print(self.joystick.getX(),self.joystick.getY(),self.joystick.getZ())
        self.driveTrain.setState(self.joystick.getY(),self.joystick.getX(),self.joystick.getZ()*2)#self.joystick.getZ()*2)
class autoDriveTrainCommand(commands2.Command):
    def __init__(self,driveSubsys:driveTrainSubsys,joySubsys:hotasSubsys):
        super().__init__()
        
    def execute(self):
        pass
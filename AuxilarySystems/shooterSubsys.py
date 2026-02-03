import commands2,wpilib,phoenix6,rev
from AuxilarySystems import auxiliaryConfig
from customFunctions import thresholdEqual

class shooterSubsys(commands2.Subsystem):
    def __init__(self):
        self.indexMotor=rev.SparkMax(auxiliaryConfig.indexMotorID,rev.SparkMax.MotorType.kBrushless)
        self.shooterMotor=phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID)
        self.RPSControl=phoenix6.controls.VelocityVoltage(0).with_slot(0)
        self.shooterMotorConfig=phoenix6.configs.TalonFXConfiguration()
        self.shooterMotorConfig.slot0.k_p=0.3
        self.shooterMotor.configurator.apply(self.shooterMotorConfig,0.5)

    def setIndexMotor(self,speed):
        self.indexMotor.set(speed)

    def setShooterRPM(self,RPM):
        self.shooterMotor.set_control(self.RPSControl.with_velocity(RPM/60))
    def getShooterRPM(self):
        return self.shooterMotor.get_velocity().value*60
class shootBalls(commands2.Command):
    def __init__(self,shooterSubsys:shooterSubsys,indexSpeed,shooterRPM):
        self.shooter=shooterSubsys
        self.speed=indexSpeed
        self.RPM=shooterRPM
    def execute(self):
        print("EXECUTINGWJAONDIAWOJDNIAWDNIANWDIUANWDIU",self.RPM)
        self.shooter.setShooterRPM(self.RPM)
        print(self.shooter.getShooterRPM())
        if thresholdEqual(self.shooter.getShooterRPM(),self.RPM,2500) or True:
            print("hhhh")
            self.shooter.setIndexMotor(self.speed)
    def end(self, interrupted):
        self.shooter.setIndexMotor(0)
        self.shooter.setShooterRPM(0)
        #NOTE NEED TO ADD ANTIJAM
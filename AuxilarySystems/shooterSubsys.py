import commands2
import wpilib
import phoenix6
from phoenix6 import configs, controls
import rev
from AuxilarySystems import auxiliaryConfig

class shooterSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.timer = wpilib.Timer()
        self.timer2 = wpilib.Timer()
        self.state = 'init'
        self.bigBoy1 = phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID1)
        self.bigBoy2 = phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID2)
        self.bigBoy3 = phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID3)
        self.bigBoy4 = phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID4)
        self.littleone = rev.SparkMax(auxiliaryConfig.shooterIndexMotorID,rev.SparkMax.MotorType.kBrushless)
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.1
        big_config.k_i = 0
        big_config.k_d = 0
        big_config.k_s = 0.3
        big_config.k_v = 0.63
        rampconfig = phoenix6.configs.ClosedLoopRampsConfigs()
        rampconfig.voltage_closed_loop_ramp_period = 0.5
        self.bigBoy1.configurator.apply(big_config)
        self.bigBoy2.configurator.apply(big_config)
        self.bigBoy3.configurator.apply(big_config)
        self.bigBoy4.configurator.apply(big_config)
        self.bigBoy1.configurator.apply(rampconfig)
        self.bigBoy2.configurator.apply(rampconfig)
        self.bigBoy3.configurator.apply(rampconfig)
        self.bigBoy4.configurator.apply(rampconfig)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller =  wpilib.XboxController(1) #wpilib.Joystick(0)
        self.brake = controls.NeutralOut()
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.toggleshoot = False
        self.RBPressed = False
        self.LBPressed = False
        self.timer2.reset()
        self.timer.reset()
        self.timer.start()
        self.targetVelocity = 2 # initial target velocity for all BigBoy's
        self.VelocityIncrement = 1 # velocity increment when performing shooting test

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'

    def periodic(self):

        if self.state == 'teleop':
            self.executeState()
        else:
            self.toggleshoot = False
            self.bigBoy1.set_control(self.brake)
            self.bigBoy2.set_control(self.brake)
            self.bigBoy3.set_control(self.brake)
            self.bigBoy4.set_control(self.brake)
            self.littleone.set(0)

    def setTargetDistance(self, distance):
        # convert distance to a target motor velocity
        # need to fit test data with polynomial
        self.targetVelocity = distance

    def executeState(self):
        
        self.prevVal = self.XPressed
        self.XPressed = self.controller.getRawAxis(3)
        self.XChanged = self.prevVal < 0.5 and self.XPressed > 0.5 or (self.prevVal > 0.5 and self.XPressed < 0.5)

        self.prevVal2 = self.LBPressed
        self.LBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityUpBtnIdx)
        self.LBChanged = self.prevVal2 == False and self.LBPressed == True

        self.prevVal3 = self.RBPressed
        self.RBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityDownBtnIdx)
        self.RBChanged = self.prevVal3 == False and self.RBPressed == True

        if self.state == 'teleop':
            if self.LBChanged:
                self.targetVelocity = (self.targetVelocity + self.VelocityIncrement)
                print (f'update target velocity to {self.targetVelocity}')
            if self.RBChanged:
                self.targetVelocity = (self.targetVelocity - self.VelocityIncrement)
                print (f'update target velocity to {self.targetVelocity}')
            
            if self.timer2 == auxiliaryConfig.shooterStartupTime:
                self.timer2.reset()
                self.littleone.set(-1 * auxiliaryConfig.shooterIndexDutyCycle)

            if self.XChanged and not self.toggleshoot:
                self.toggleshoot = True
                self.bigBoy1.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
                self.bigBoy2.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
                self.bigBoy3.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
                self.bigBoy4.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
                self.littleone.set(auxiliaryConfig.shooterIndexDutyCycle)
                self.timer2.start()
                print ('starting all shooter motors')
            elif self.XChanged and self.toggleshoot:
                self.toggleshoot = False
                print ('stopping all shooter motors')
                self.bigBoy1.set_control(self.brake)
                self.bigBoy2.set_control(self.brake)
                self.bigBoy3.set_control(self.brake)
                self.bigBoy4.set_control(self.brake)
                self.littleone.set(0)

            if self.timer.get() > .99 :
                if self.toggleshoot :
                    print(f'current shooter velocity:{self.bigBoy1.get_velocity().value}, target velocity {self.targetVelocity}')
                self.timer.reset()
                self.timer.start()           
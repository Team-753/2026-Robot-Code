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
        rampconfig.voltage_closed_loop_ramp_period = 0.8
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
        #self.XChanged = False
        self.XStart = False
        self.XStop = False
        self.LBChanged = False
        self.RBChanged = False
        self.toggleshoot = False
        self.RBPressed = False
        self.LBPressed = False
        self.autoFeedEnabled = False
        self.timer2.reset()
        self.timer.reset()
        self.timer.start()
        self.manualTargetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
        self.autoTargetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
        self.targetVelocity = auxiliaryConfig.shooterDefaultVelocityRps
        self.VelocityIncrement = auxiliaryConfig.shooterVelocityIncrementRps
        self.autoVelocityEnabled = True
        self.targetDistanceMeters = None

    def teleopInit(self):
        self.state = 'teleop'
        self.autoFeedEnabled = False
        self.refreshTargetVelocity()

    def autoInit(self):
        self.state = 'auto'
        self.XPressed = False
        self.prevVal = False
        self.XStart = False
        self.XStop = False
        self.LBChanged = False
        self.RBChanged = False
        self.RBPressed = False
        self.LBPressed = False
        self.toggleshoot = False
        self.autoFeedEnabled = False
        self.timer2.stop()
        self.timer2.reset()
        self.bigBoy1.set_control(self.brake)
        self.bigBoy2.set_control(self.brake)
        self.bigBoy3.set_control(self.brake)
        self.bigBoy4.set_control(self.brake)
        self.littleone.set(0)
        self.refreshTargetVelocity()

    def setToIdle(self):
        self.state = 'idle'

    def setAutoVelocityEnabled(self, enabled):
        enabled = bool(enabled)
        if self.autoVelocityEnabled != enabled:
            self.autoVelocityEnabled = enabled
            mode = 'auto' if enabled else 'manual'
            print(f'shooter velocity mode set to {mode}')
        self.refreshTargetVelocity()

    def calculateVelocityForDistance(self, distance):
        distance = max(0.0, distance)
        ratio = auxiliaryConfig.shooterVelocityReferenceRps / auxiliaryConfig.shooterVelocityReferenceDistanceMeters
        return pow(ratio * distance,1.05)

    def refreshTargetVelocity(self):
        if self.autoVelocityEnabled:
            self.targetVelocity = self.autoTargetVelocity
        else:
            self.targetVelocity = self.manualTargetVelocity

    def applyTargetVelocity(self):
        self.bigBoy1.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
        self.bigBoy2.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
        self.bigBoy3.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))
        self.bigBoy4.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0.2))

    def isShooting(self):
        return self.toggleshoot

    def autoShootStart(self):
        if self.state == 'auto' and not self.toggleshoot:
            #self.XChanged = True
            self.XStart = True
            print('enabling shooter from auto')

    def autoFeedStart(self):
        if self.state == 'auto' and self.toggleshoot and not self.autoFeedEnabled:
            self.autoFeedEnabled = True
            self.littleone.set(-1 * auxiliaryConfig.shooterIndexDutyCycle)
            print('enabling shooter loader from auto')

    def autoFeedStop(self):
        if self.state == 'auto' and self.autoFeedEnabled:
            self.autoFeedEnabled = False
            self.littleone.set(0)
            print('disabling shooter loader from auto')
    
    def autoShootStop(self):
        if self.state == 'auto':
            self.XStart = False
            self.XStop = False
            self.toggleshoot = False
            self.autoFeedEnabled = False
            self.bigBoy1.set_control(self.brake)
            self.bigBoy2.set_control(self.brake)
            self.bigBoy3.set_control(self.brake)
            self.bigBoy4.set_control(self.brake)
            self.littleone.set(0)
            self.timer2.stop()
            self.timer2.reset()
            print('disabling shooter from auto')

    def periodic(self):

        if self.state == 'teleop':
            # update inputs from user buttons
            self.prevVal = self.XPressed
            self.XPressed = self.controller.getRawAxis(auxiliaryConfig.shooterEnableBtnIdx)
            #self.XChanged = (self.prevVal < 0.5 and self.XPressed > 0.5) or (self.prevVal > 0.5 and self.XPressed < 0.5)
            self.XStart = self.prevVal < 0.5 and self.XPressed > 0.5
            self.XStop = self.prevVal > 0.5 and self.XPressed < 0.5

            self.prevVal2 = self.LBPressed
            self.LBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityUpBtnIdx)
            self.LBChanged = self.prevVal2 == False and self.LBPressed == True

            self.prevVal3 = self.RBPressed
            self.RBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityDownBtnIdx)
            self.RBChanged = self.prevVal3 == False and self.RBPressed == True

            # execute drive
            self.executeState()

        elif self.state == 'auto':
            # no button input needed. programatically set XChanged

            # execute drive
            self.executeState()

            # make sure to set XChanged back to false after execution
            #self.XChanged = False
            self.XStart = False
            self.XStop = False

        else:
            self.toggleshoot = False
            self.autoFeedEnabled = False
            self.bigBoy1.set_control(self.brake)
            self.bigBoy2.set_control(self.brake)
            self.bigBoy3.set_control(self.brake)
            self.bigBoy4.set_control(self.brake)
            self.littleone.set(0)
            self.timer2.stop()
            self.timer2.reset()

    def setTargetDistance(self, distance):
        self.targetDistanceMeters = distance
        self.autoTargetVelocity = self.calculateVelocityForDistance(distance)
        self.refreshTargetVelocity()

    def executeState(self):
    
        # for testing only, update target velocity based on user buttons
        if self.LBChanged:
            self.manualTargetVelocity = self.manualTargetVelocity + self.VelocityIncrement
            self.refreshTargetVelocity()
            print (f'update manual target velocity to {self.manualTargetVelocity}')
        if self.RBChanged:
            self.manualTargetVelocity = max(0, self.manualTargetVelocity - self.VelocityIncrement)
            self.refreshTargetVelocity()
            print (f'update manual target velocity to {self.manualTargetVelocity}')
        
        # delay loader motor from starting for shooterStartupTime seconds
        if self.state != 'auto' and self.timer2.get() >= auxiliaryConfig.shooterStartupTime:
            self.timer2.stop()
            self.timer2.reset()
            self.littleone.set(-1 * auxiliaryConfig.shooterIndexDutyCycle)
            print('loader activated')

        # start shooter
        if self.XStart and not self.toggleshoot: # self.XChanged and not self.toggleshoot:
            self.toggleshoot = True
            self.refreshTargetVelocity()
            self.applyTargetVelocity()
            self.littleone.set(0)
            if self.state != 'auto':
                self.timer2.reset()
                self.timer2.start()
            print ('starting all shooter motors')

        # stop shooter
        elif self.XStop and self.toggleshoot: #self.XChanged and self.toggleshoot:
            self.toggleshoot = False
            self.autoFeedEnabled = False
            print ('stopping all shooter motors')
            self.bigBoy1.set_control(self.brake)
            self.bigBoy2.set_control(self.brake)
            self.bigBoy3.set_control(self.brake)
            self.bigBoy4.set_control(self.brake)
            self.littleone.set(0)
            self.timer2.stop()
            self.timer2.reset()

        if self.toggleshoot:
            self.refreshTargetVelocity()
            self.applyTargetVelocity()

        # read velocity for diagnostics
        if self.timer.get() > .99 :
            if self.toggleshoot :
                print(f'current shooter velocity:{self.bigBoy1.get_velocity().value}, target velocity {self.targetVelocity}')
            self.timer.reset()
            self.timer.start()      

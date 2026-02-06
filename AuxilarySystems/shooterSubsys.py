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
        self.state = 'init'
        self.bigBoy = phoenix6.hardware.TalonFX(auxiliaryConfig.shooterMotorID)
        self.littleone = rev.SparkMax(auxiliaryConfig.indexMotorID,rev.SparkMax.MotorType.kBrushless)
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.25
        big_config.k_i = 0
        big_config.k_d = 0
        #big_config.k_s = 0.1
        #big_config.k_v = 0.15
        self.bigBoy.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller = wpilib.Joystick(0) # wpilib.XboxController(0)
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.toggleshoot = False
        self.RBPressed = False
        self.LBPressed = False
        self.timer.reset()
        self.timer.start()
        self.targetVelocity = 10

    def teleopInit(self):
        self.state = 'teleop'

    def periodic(self):

        if self.state == 'teleop':
            self.executeState()
        else:
            self.toggleshoot = False
            self.bigBoy.set_control(self.request.with_velocity(0))
            self.littleone.set(0)



    def executeState(self):
        
        self.prevVal = self.XPressed
        self.XPressed = self.controller.getRawButton(auxiliaryConfig.shooterEnableBtnIdx) #.getXButton()
        self.XChanged = self.prevVal == False and self.XPressed == True

        self.prevVal2 = self.LBPressed
        self.LBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityUpBtnIdx) #getLeftBumperButton()
        self.LBChanged = self.prevVal2 == False and self.LBPressed == True

        self.prevVal3 = self.RBPressed
        self.RBPressed = self.controller.getRawButton(auxiliaryConfig.shooterVelocityDownBtnIdx) #getRightBumperButton()
        self.RBChanged = self.prevVal3 == False and self.RBPressed == True

        if self.state == 'teleop':
            if self.LBChanged:
                self.targetVelocity = (self.targetVelocity + 5)
                print (self.targetVelocity)
            if self.RBChanged:
                self.targetVelocity = (self.targetVelocity - 5)
                print (self.targetVelocity)
            if self.XChanged and not self.toggleshoot:
                self.toggleshoot = True
                self.bigBoy.set_control(self.request.with_velocity(self.targetVelocity))
                self.littleone.set(0.5)
                print ('true')
            elif self.XChanged and self.toggleshoot:
                self.toggleshoot = False
                print ('False')
                self.bigBoy.set_control(self.request.with_velocity(0))
                self.littleone.set(0)
            if self.timer.get() > .99 :
                print(f'current shooter velocity:{self.bigBoy.get_velocity().value}, target velocity {self.targetVelocity}')
                self.timer.reset()
                self.timer.start()            

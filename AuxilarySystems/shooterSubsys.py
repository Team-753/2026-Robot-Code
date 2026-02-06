import commands2
import wpilib
import phoenix6
from phoenix6 import configs, controls
import rev


class shooterSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.timer = wpilib.Timer()
        self.state = 'init'
        self.bigBoy = phoenix6.hardware.TalonFX(0)
        self.littleone = rev.SparkMax(16,rev.SparkMax.MotorType.kBrushless)
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.3
        big_config.k_i = 0
        big_config.k_d = 0
        #big_config.k_s = 0.1
        big_config.k_v = 0.15
        self.bigBoy.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller = wpilib.Joystick(2) # wpilib.XboxController(0)
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.toggleshoot = False
        self.RBPressed = False
        self.LBPressed = False
        self.timer.reset()
        self.timer.start()
        self.targetVelocity = 50

    def teleopInit(self):
        self.state = 'teleop'

    def periodic(self):

        if self.state == 'teleop':
            self.executeState()
        else:
            self.toggleshoot = False
            self.bigBoy.set_control(self.request.with_velocity(0).with_feed_forward(0))
            self.littleone.set(0)



    def executeState(self):
        
        self.prevVal = self.XPressed
        self.XPressed = self.controller.getRawButton(3) #.getXButton()
        self.XChanged = self.prevVal == False and self.XPressed == True

        self.prevVal2 = self.LBPressed
        self.LBPressed = self.controller.getRawButton(6) #getLeftBumperButton()
        self.LBChanged = self.prevVal2 == False and self.LBPressed == True

        self.prevVal3 = self.RBPressed
        self.RBPressed = self.controller.getRawButton(4) #getRightBumperButton()
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
                self.bigBoy.set_control(self.request.with_velocity(self.targetVelocity).with_feed_forward(0))
                self.littleone.set(0.5)
                print ('true')
            elif self.XChanged and self.toggleshoot:
                self.toggleshoot = False
                print ('False')
                self.bigBoy.set_control(self.request.with_velocity(0).with_feed_forward(0))
                self.littleone.set(0)
            if self.timer.get() > .99 :
                print(f'current velocity:{self.bigBoy.get_velocity().value}')
                self.timer.reset()
                self.timer.start()            

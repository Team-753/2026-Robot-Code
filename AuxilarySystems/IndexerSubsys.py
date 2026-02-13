import commands2
import wpilib
import phoenix6
from phoenix6 import configs, controls
import rev
from AuxilarySystems import auxiliaryConfig


class indexerSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.timer = wpilib.Timer()
        self.state = 'init'
        self.numberOne = phoenix6.hardware.TalonFX(auxiliaryConfig.indexerMotorIndexNumberLIKETHEONLYMOTOR)
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.3
        big_config.k_i = 0
        big_config.k_d = 0
        #big_config.k_s = 0.1
        big_config.k_v = 0.15
        self.numberOne.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller =   wpilib.XboxController(0) #wpilib.Joystick(2)
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.toggleshoot = False
        self.timer.reset()
        self.timer.start()

    def teleopInit(self):
        self.state = 'teleop'

    def periodic(self):

        if self.state == 'teleop':
            self.executeState()
        else:
            self.toggleshoot = False
            self.numberOne.set_control(self.request.with_velocity(0).with_feed_forward(0))



    def executeState(self):
        
        self.prevVal = self.XPressed
        self.XPressed = self.controller.getXButton() #.getXButton()
        self.XChanged = self.prevVal == False and self.XPressed == True

        if self.state == 'teleop':
           
            if self.XChanged and not self.toggleshoot:
                self.toggleshoot = True
                self.numberOne.set_control(self.request.with_velocity(auxiliaryConfig.indexerSpeed).with_feed_forward(0))
                print ('true')
            elif self.XChanged and self.toggleshoot:
                self.toggleshoot = False
                print ('False')
                self.numberOne.set_control(self.request.with_velocity(0).with_feed_forward(0))
            if self.timer.get() > .99 :
                print(f'current velocity:{self.numberOne.get_velocity().value}')
                self.timer.reset()
                self.timer.start()            

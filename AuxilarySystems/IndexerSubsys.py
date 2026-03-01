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
        self.timer2 = wpilib.Timer()
        self.state = 'init'
        self.numberOne = rev.SparkMax(auxiliaryConfig.indexerMotorIndexNumberLIKETHEONLYMOTOR,rev.SparkMax.MotorType.kBrushless)
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = .11
        big_config.k_i = 0
        big_config.k_d = 0
        big_config.k_s = 0.3
        big_config.k_v = 0.63
        #self.numberOne.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller = wpilib.XboxController(1) #wpilib.Joystick(2)
        self.brake = controls.NeutralOut()
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        #CHRIS MOD
        self.BPressed = False
        self.prevVal2 = False
        self.BChanged = False
        #END CHRIS MOD
        self.toggleshoot = False
        self.waiting = False
        self.timer.reset()
        self.timer.start()

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'
        
    def setToIdle(self):
        self.state = 'idle'

    def autoShootStart(self):
        if self.state == 'auto' and not self.toggleshoot:
            self.XChanged = True
            print('enabling shooter from auto')
    
    def autoShootStop(self):
        if self.state == 'auto' and self.toggleshoot:
            self.XChanged = True
            print('disabling shooter from auto')

    def periodic(self):

        if self.state == 'teleop':
            self.prevVal = self.XPressed
            self.XPressed = self.controller.getRawAxis(auxiliaryConfig.indexerEnableBtnIdx)
            self.XChanged = (self.prevVal < 0.5 and self.XPressed > 0.5) or (self.prevVal > 0.5 and self.XPressed < 0.5)
            #CHRIS MOD
            self.prevVal2 = self.BPressed
            self.BPressed = self.controller.getRawButton(auxiliaryConfig.intakeSpinEnableBtnIdx)
            self.BChanged = self.BChanged = self.prevVal2 == False and self.BPressed == True
            #CHRIS MOD END
            self.executeState()
        elif self.state == 'auto':
            self.executeState()
            self.XChanged = False
        else:
            self.toggleshoot = False
            self.numberOne.set(0) 

    def executeState(self):
        
        
        #MODIFIED "self.XChanged" ---> "(self.XChanged or self.BChanged)"
        if (self.XChanged or self.BChanged) and not self.toggleshoot:
            self.toggleshoot = True
            self.waiting = True
            self.timer2.start()
            print('waiting for big motor speed')
            
        elif (self.XChanged or self.BChanged) and self.toggleshoot:
            self.toggleshoot = False
            self.waiting = False
            self.timer2.reset()
            print ('Indexer stopping motor')
            self.numberOne.set(0) 
        
        if self.timer.get() > .99 :
            #print(f'current velocity:{self.numberOne.get_velocity().value}')
            self.timer.reset()
            self.timer.start()            

        if self.timer2.get() >= auxiliaryConfig.shooterStartupTime and self.waiting == True:
            self.numberOne.set(auxiliaryConfig.indexerSpeed)
            print ('indexer starting motor')
            self.waiting = False
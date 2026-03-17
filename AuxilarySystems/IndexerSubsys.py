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
        self.XStart = False 
        self.XStop = False
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.intakeRunning = False
        self.shooterRunning = False
        self.indexerLogic = False
        self.XStart = False
        self.XStop = False
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
        self.XStart = False
        self.XStop = False
        self.XPressed = False
        self.prevVal = False
        self.XChanged = False
        self.intakeRunning = False
        self.shooterRunning = False
        self.indexerLogic = False
        self.BPressed = False
        self.prevVal2 = False
        self.BChanged = False
        self.toggleshoot = False
        self.waiting = False
        self.numberOne.set(0)
        
    def setToIdle(self):
        self.state = 'idle'

    def autoShootStart(self):
        if self.state == 'auto' and not self.toggleshoot:
            self.XStart = True
            print('enabling indexer from auto')
    
    def autoShootStop(self):
        if self.state == 'auto':
            self.XStart = False
            self.XStop = False
            self.intakeRunning = False
            self.shooterRunning = False
            self.indexerLogic = False
            self.toggleshoot = False
            self.numberOne.set(0)
            print('disabling indexer from auto')

    def periodic(self):

        if self.state == 'teleop':
            self.prevVal = self.XPressed
            self.XPressed = self.controller.getRawAxis(auxiliaryConfig.indexerEnableBtnIdx)
            self.XStart = self.prevVal < 0.5 and self.XPressed > 0.5
            self.XStop = self.prevVal > 0.5 and self.XPressed < 0.5
            #CHRIS MOD
            self.prevVal2 = self.BPressed
            self.BPressed = self.controller.getRawButton(auxiliaryConfig.intakeSpinEnableBtnIdx)
            self.BChanged = self.prevVal2 == False and self.BPressed == True
            #CHRIS MOD END
            self.executeState()
        elif self.state == 'auto':
            self.executeState()
            self.XStart = False
            self.XStop = False
        else:
            self.toggleshoot = False
            self.intakeRunning = False
            self.indexerLogic = False
            self.shooterRunning = False
            self.numberOne.set(0) 

    def executeState(self):

        if self.BChanged:
            # print('intake toggled')
            self.intakeRunning = not self.intakeRunning

        if self.XStart and not self.shooterRunning:
            print('shooter enabled')
            self.shooterRunning = True
        if self.XStop and self.shooterRunning:
            print('shooter disabled')
            self.shooterRunning = False

        self.LogicPrevVal = self.indexerLogic

        if self.shooterRunning:#or self.intakeRunning:
            self.indexerLogic = True
        
        if not self.shooterRunning:# and not self.intakeRunning:
            self.indexerLogic = False
        
        self.indexerToggle = self.LogicPrevVal != self.indexerLogic

        if self.indexerToggle and self.indexerLogic:
            print ('indexer starting motor')
            self.numberOne.set(auxiliaryConfig.indexerSpeed)
            self.toggleshoot= True

        elif self.indexerToggle and not self.indexerLogic:
            print('indexer stopping motor')
            self.toggleshoot= False
            self.numberOne.set(0)
                
        # #MODIFIED "self.XChanged" ---> "(self.XChanged or self.BChanged)"
        # if (self.XStart or self.BChanged) and not self.toggleshoot:
        #     self.toggleshoot = True
        #     self.intakeRunning = self.BChanged
        #     self.numberOne.set(auxiliaryConfig.indexerSpeed)
        #     print ('indexer starting motor')
            
        # elif self.XStop and self.toggleshoot and not self.intakeRunning:
        #     self.toggleshoot = False
        #     print ('Indexer stopping motor')
        #     self.numberOne.set(0) 
        
        # elif self.BChanged and self.intakeRunning and self.toggleshoot:
        #     self.intakeRunning = False
        #     self.toggleshoot = False
        #     self.numberOne.set(0)
        #     print('indexer stopping motor')
             
        
        if self.timer.get() > .99 :
            #print(f'current velocity:{self.numberOne.get_velocity().value}')
            self.timer.reset()
            self.timer.start()

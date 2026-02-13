import commands2
import phoenix6
import wpilib
from phoenix6 import configs, controls
from collections.abc import Sequence
from AuxilarySystems import auxiliaryConfig

class flipsubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.armmotor = phoenix6.hardware.TalonFX(1)
        self.grabermotor=phoenix6.hardware.TalonFX(2)
        Turn_configs = phoenix6.configs.Slot0Configs()
        Turn_configs.k_s = 0.1
        Turn_configs.k_v = 1
        Turn_configs.k_p = .22
        Turn_configs.k_i = 0
        Turn_configs.k_d = 0
        go_configs = phoenix6.configs.Slot1Configs()
        go_configs.k_s = 0.1
        go_configs.k_v = 2
        go_configs.k_p = .22
        go_configs.k_i = 0
        go_configs.k_d = 0
        self.armmotor.configurator.apply(go_configs)
        self.grabermotor.configurator.apply(Turn_configs)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.requests = controls.PositionVoltage(0).with_slot(1)
        self.controller = wpilib.XboxController(0)
        self.state = 'init'
        self.substate = 'none'
        self.homingstate = 'start'
        self.homepointarm = 'set'
        self.enabled = False
        self.positionset = 1.0
        self.position2set = -1.0
        self.pos = 0
        self.target = 0
        self.limitswitch1 = wpilib.DigitalInput(0)
        self.limitswitch1pressesed = True
        self.limitswitch2pressesed = True

    def home1(self):
        if self.homingstate == 'start' :
            self.armmotor.set_control(self.request.with_velocity(3).with_feed_forward(0.5))
            print("2 more")
            self.homingstate = 'waitForLimit'

        elif self.homingstate == 'waitForLimit' :
            if self.limitswitch1pressesed == False:
                self.armmotor.set_control(self.request.with_velocity(0).with_feed_forward(0))
                print("1 more")
                self.homingstate = 'complete'   

        elif self.homingstate == 'complete':
            self.armmotor.set_position(0)
            print("0 more")
            self.homingstate = 'start'
            self.substate = 'none'

    def home2(self):
        if self.homingstate == 'start' :
            self.grabermotor.set_control(self.request.with_velocity(3).with_feed_forward(0))
            print("dos mas")
            self.homingstate = 'waitForLimit'

        elif self.homingstate == 'waitForLimit' :
            if self.limitswitch2pressesed:
                self.grabermotor.set_control(self.request.with_velocity(0).with_feed_forward(0))
                print("uno mas")
                self.homingstate = 'complete'   

        elif self.homingstate == 'complete':
            self.grabermotor.set_position(0)
            print("zero mas")
            self.homingstate = 'start'
            self.substate = 'none'

    def go_down(self):
        self.grabermotor.set_control(self.requests.with_position(9))
        pos =self.grabermotor.get_position()
        if pos <= 9:
            self.getout = 'done'
    
    def position0(self):
        if self.homepointarm == 'start':
            self.grabermotor.set_control(self.requests.with_position(0))
            self.homepointarm = 'waitforamoment'
        elif self.homepointarm == 'waitforamoment':
           self.pos = self.armmotor.get_position()
           if self.target < (self.pos.value + .05) or self.target > (self.pos.value - 0.05):
               self.armmotor.set_control(self.requests.with_position(0))
               self.homepointarm = 'done'
        elif self.homepointarm == 'done':   
            self.pos = self.armmotor.get_position()
            if self.pos < (self.pos.value + .05) or self.pos > (self.pos.value - .05):
                    print("all done")             

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'

    def periodic(self):
        if self.state == 'teleop': 
            APressed = self.controller.getAButton()
            BPressed = self.controller.getBButton()
            Homepressed = self.controller.getXButton()
            Homepressed2 = self.controller.getYButton()
            LeftBumberPressed = self.controller.getLeftBumper()
            RightBumberPressed = self.controller.getRightBumper()
            self.limitswitch1pressesed = self.limitswitch1.get()
            self.limitswitch2pressesed = self.controller.getRightBumperButton()
            self.pos = self.armmotor.get_position()

            if BPressed and BPressed != self.enabled :
                self.enabled = BPressed 
                self.target = 0
                self.pos = self.armmotor.get_position()
                if self.target < (self.pos.value + .05) or self.target > (self.pos.value - 0.05):
                    self.armmotor.set_control(self.requests.with_position(10).with_feed_forward(0))
                    self.target = 0
                self.enabled = False

            elif APressed and APressed != self.enabled :
                self.enabled = APressed
                
                self.armmotor.set_control(self.requests.with_position(0).with_feed_forward(0))

            elif LeftBumberPressed and LeftBumberPressed != self.enabled :
                self.position0()

            elif Homepressed :
                self.substate = 'homing'

            elif Homepressed2 :
                self.substate = 'homing2'

            elif self.substate == 'homing' :
                self.home1()

            elif self.substate == 'homing2':
                self.home2()

            else :
                self.pos = self.armmotor.get_position()
                #print(self.pos)
                if self.target < (self.pos.value + .05) or self.target > (self.pos.value - 0.05):
                      self.enabled = False
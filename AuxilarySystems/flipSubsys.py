import commands2
import phoenix6
import wpilib
from phoenix6 import configs, controls
from collections.abc import Sequence
import rev
from AuxilarySystems import auxiliaryConfig


class flipsubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.armmotor = rev.SparkMax(auxiliaryConfig.flipMotorID,rev.SparkMax.MotorType.kBrushless)
        self.grabermotor=phoenix6.hardware.TalonFX(2)
        self.encoder = self.armmotor.getEncoder()

        graberconfigs = phoenix6.configs.Slot0Configs()
        graberconfigs.k_s = auxiliaryConfig.graber_k_s_config
        graberconfigs.k_v = auxiliaryConfig.graber_k_v_config
        graberconfigs.k_p = auxiliaryConfig.graber_k_p_config
        graberconfigs.k_i = auxiliaryConfig.graber_k_i_config
        graberconfigs.k_d = auxiliaryConfig.graber_k_d_config


        self.armconfigs = rev.SparkMaxConfig()
        self.armconfigs.closedLoop.pid(auxiliaryConfig.arm_k_p_config)

        self.armmotor.configure(self.armconfigs,rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.grabermotor.configurator.apply(graberconfigs)

        self.grabermotor.set_position(0)

        self.armrequests = controls.VelocityVoltage(0).with_slot(0)
        self.graberrequests = controls.PositionVoltage(0).with_slot(1)
        self.controller = wpilib.XboxController(0)

        self.state = 'init'
        self.substate = 'none'
        self.homingstate = 'start'
        self.homepointarm = 'set'
        self.armout = 'start'
        self.lv1flipgo = 'start'
        self.lv3flipgo = 'start'

        self.graberout = 1
        self.armoutpos = 10
        self.enabled = False
        self.positionset = 1.0
        self.position2set = -1.0
        self.target = 0

        self.limitswitch1 = wpilib.DigitalInput(0)
        self.limitswitch1pressesed = True
        self.limitswitch2pressesed = True

    def home(self):
        if self.homingstate == 'start' :
            self.armmotor.set_control(self.armrequests.with_velocity(auxiliaryConfig.armhomespeed).with_feed_forward(0))
            self.homingstate = 'waitForLimit'

        elif self.homingstate == 'waitForLimit' :
            if self.limitswitch1pressesed == False:
                self.armmotor.set_control(self.armrequests.with_velocity(0).with_feed_forward(0))
                self.homingstate = 'complete'   

        elif self.homingstate == 'complete':
                self.armmotor.set_position(0)
                self.homingstate = 'startagain'

    def go_down(self):
        self.grabermotor.set_control(self.graberrequests.with_position(self.graberout))
        pos =self.grabermotor.get_position()
        if pos == self.graberout:
            self.getout = 'done'
       
    #can't stop
    def position_home(self):
        if self.homepointarm == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(0))
            self.homepointarm = 'waitforamoment'
        elif self.homepointarm == 'waitforamoment':
           self.pos = self.encoder.getPosition()
           if self.target < (self.pos + .05) or self.target > (self.pos - 0.05):
               target_rotations = -self.armoutpos
               self.armconfigs.setReference(target_rotations, rev.SparkMax.ControlType.kPosition)
               self.homepointarm = 'done'
  

        elif self.homepointarm == 'done':   
            self.armpos = self.encoder.getPosition()
            armtarget = 0
            if armtarget < (self.armpos + .05) or armtarget > (self.armpos - .05):
                    print("arm_in.")
        
    def position_out(self):
        if self.armout == 'start':
            target_rotations = self.armoutpos
            self.armconfigs.setReference(target_rotations, rev.SparkMax.ControlType.kPosition)
            self.armout = 'wait'
        elif self.armout == 'wait':
            armpos= self.encoder.getPosition()
            armtarget = self.armoutpos
            if armtarget < (armpos + .05) or armtarget > (armpos - 0.05):
                self.grabermotor.set_control(self.graberrequests.with_position(self.graberout).with_feed_forward(0))
                grabbertarget = self.graberout
                graberpos = self.grabermotor.get_position()
                if grabbertarget < (graberpos + .05) or grabbertarget > (graberpos - 0.05):
                    print("arm out. Ready for climb.")
  
    def lv1flip(self):
        if self.lv1flipgo == 'start':
             self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv1).with_feed_forward(0))
  

    def lv3flip(self):
        if self.lv3flipgo == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv3).with_feed_forward(0))
           

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'


    def periodic(self):
        APressed = self.controller.getAButton()
        BPressed = self.controller.getBButton()
        Homepressed = self.controller.getXButton()
        go_downPressed = self.controller.getYButton()
        Go_homePressed = self.controller.getLeftBumper()
        lv3flipPressed = self.controller.getRightBumper()
        self.limitswitch1pressesed = self.limitswitch1.get()
        self.limitswitch2pressesed = self.controller.getRightBumperButton()

        if self.state == 'teleop': 

            if APressed and APressed != self.enabled :
                self.position_home()
                
            elif Go_homePressed and Go_homePressed != self.enabled :
                self.position_out()

            elif Homepressed and Homepressed != self.enabled :
                self.home()

            elif go_downPressed and go_downPressed != self.enabled :
                self.go_down()

            elif lv3flipPressed and lv3flipPressed != self.enabled:
                self.lv3flip()

        if self.state == 'auto':
            pass
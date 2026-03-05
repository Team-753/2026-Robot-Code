import rev
import commands2
import phoenix6
import wpilib
from phoenix6 import configs, controls
from collections.abc import Sequence
from AuxilarySystems import auxiliaryConfig


class flipsubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.armmotor = rev.SparkMax(auxiliaryConfig.flipMotorID,rev.SparkMax.MotorType.kBrushless)
        self.grabermotor=phoenix6.hardware.TalonFX(2)
        self.encoder = self.armmotor.getEncoder()
        self.armmotorController = self.armmotor.getClosedLoopController()

        graberconfigs = phoenix6.configs.Slot0Configs()
        graberconfigs.k_s = auxiliaryConfig.graber_k_s_config
        graberconfigs.k_v = auxiliaryConfig.graber_k_v_config
        graberconfigs.k_p = auxiliaryConfig.graber_k_p_config
        graberconfigs.k_i = auxiliaryConfig.graber_k_i_config
        graberconfigs.k_d = auxiliaryConfig.graber_k_d_config


        self.armconfigs = rev.SparkMaxConfig()
        arm_k_p_config=rev.SparkMaxConfig() 
        arm_k_p_config.closedLoop.P(0.1)
        arm_k_p_config.closedLoop.I(0.0)
        arm_k_p_config.closedLoop.D(0.0)

        self.armmotor.configure(self.armconfigs,rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.grabermotor.configurator.apply(graberconfigs)

        self.armrequests = controls.VelocityVoltage(0).with_slot(0)
        self.graberrequests = controls.PositionVoltage(0).with_slot(1)
        self.controller = wpilib.XboxController(4)

        self.state = 'init'
        self.substate = 'none'
        self.homingstate = 'start'
        self.homepointarm = 'set'
        self.armout = 'start'
        self.lv1flipgo = 'start'
        self.lv3flipgo = 'start'

        self.graberout = 0
        self.armoutpos = 10
        self.enabled = False
        self.positionset = 1.0
        self.position2set = -1.0
        self.target = 0
        
        self.INPressed = False
        self.prevValIN = False
        self.INStart = False
        self.INStop = False

        self.OutPressed = False
        self.prevValOut = False
        self.INStart = False
        self.INStop = False

        self.RightPressed = False
        self.prevValRight = False
        self.INStart = False
        self.INStop = False

        self.LeftPressed = False
        self.prevValLeft = False
        self.INStart = False
        self.INStop = False

    #def home(self):
     #   if self.homingstate == 'start' :
      #      target_rotations_per_minute = 1
       #     self.armmotorController.setSetpoint(target_rotations_per_minute, rev.SparkMax.ControlType.kVelocity, rev.ClosedLoopSlot(0))
        #    self.homingstate = 'waitForLimit'
         #   print("I'm going home.")

#        elif self.homingstate == 'waitForLimit' :
#            if self.limitswitch1pressesed == False:
 #               target_rotations_per_minute=0
  #              self.armmotorController.setSetpoint(target_rotations_per_minute, rev.SparkMax.ControlType.kVelocity, rev.ClosedLoopSlot(0))
   #             self.homingstate = 'complete' 
    #            print("Tell the world I'm going home.")  
#
 #       elif self.homingstate == 'complete':
  #              self.encoder.setPosition(0)
   #             self.homingstate = 'startagain'
    #            print("I'm home.")
     #s           self.enabled = False

    def go_down(self):
        self.grabermotor.set_control(self.graberrequests.with_position(self.graberout))
        print("HELP")
        pos = self.grabermotor.get_position()
        if self.graberout < (pos.value + .05) or self.graberout > (pos.value - 0.05):
            self.getout = 'done'
            print("I'm down")
            self.enabled = False

       
    def position_home(self):
        if self.homepointarm == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(0))
            print("going in.")
            self.homepointarm = 'waitforamoment'
        elif self.homepointarm == 'waitforamoment':
           self.pos = self.grabermotor.get_position()
           self.target = 0
           if self.target < (self.pos.value + .05) or self.target > (self.pos.value - 0.05):
               target_rotations = 0
               self.armmotorController.setSetpoint(target_rotations, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot(0))
               self.homepointarm = 'done'
               print ("almost in")
  
        elif self.homepointarm == 'done':   
            self.armpos = self.encoder.getPosition()
            armtarget = 0
            if armtarget < (self.armpos + .05) or armtarget > (self.armpos - .05):
                    print("arm_in.")
        
    def position_out(self):
        if self.armout == 'start':
            target_rotations = self.armoutpos
            self.armmotorController.setSetpoint(target_rotations, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot(0))
            print("going out.")
            self.armout = 'wait'
        elif self.armout == 'wait':
            armpos= self.encoder.getPosition()
            armtarget = self.armoutpos
            if armtarget < (armpos + .05) or armtarget > (armpos - 0.05):
                self.grabermotor.set_control(self.graberrequests.with_position(self.graberout).with_feed_forward(0))
                grabbertarget = self.graberout
                graberpos = self.grabermotor.get_position()
                if grabbertarget < (graberpos.value + .05) or grabbertarget > (graberpos.value - 0.05):
                    print("arm out. Ready for climb.")
                    self.enabled = False
  
    def lv1flip(self):
        if self.lv1flipgo == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv1).with_feed_forward(0))
            targetpos = auxiliaryConfig.graberlv1
            pos = self.grabermotor.get_position()
            if targetpos < (pos.value + .05) or targetpos > (pos.value - 0.05):
                print("lv1")
                self.enabled = False
  

    def lv3flip(self):
        if self.lv3flipgo == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv3).with_feed_forward(0))
            targetpos = auxiliaryConfig.graberlv3
            pos = self.grabermotor.get_position()
            if targetpos < (pos.value + .05) or targetpos > (pos.value - 0.05):
                print("lv3")
                self.enabled = False
           

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'

    def testexacute(self):
        donePressed = self.controller.getLeftBumperButtonPressed()
        INSpeed = 1
        LeftSpeed = 1

        self.prevValIN = self.INPressed
        self.INPressed = self.controller.getRawButton(auxiliaryConfig.FLipINButtonINdex)
        self.INStart = self.prevValIN == False and self.INPressed == True
        self.INStop = self.prevValIN == True and self.INPressed == False

        self.prevValOut = self.OutPressed
        self.OutPressed = self.controller.getRawButton(auxiliaryConfig.FLipOutButtonINdex)
        self.OutStart = self.prevValOut == False and self.OutPressed == True
        self.OutStop = self.prevValOut == True and self.OutPressed == False

        self.prevValLeft = self.LeftPressed
        self.LeftPressed = self.controller.getRawButton(auxiliaryConfig.FLipLeftButtonINdex)
        self.LeftStart = self.prevValLeft == False and self.LeftPressed == True
        self.LeftStop = self.prevValLeft == True and self.LeftPressed == False

        self.prevValRight = self.RightPressed
        self.RightPressed = self.controller.getRawButton(auxiliaryConfig.FLipRightButtonINdex)
        self.RightStart = self.prevValRight == False and self.RightPressed == True
        self.RightStop = self.prevValRight == True and self.RightPressed == False

        if self.INStart:
            self.armmotor.set(INSpeed)
            print('Moving in')

        elif self.INStop:
            self.armmotor.set(0)
            print('not Moving in')

        elif self.OutStop:
            self.armmotor.set(0)
            print('not Moving out')

        elif self.OutStart:
            self.armmotor.set(-INSpeed)
            print('Moving out')

        elif self.RightStart:
            self.grabermotor.set(-LeftSpeed)
            print('Moving Right')

        elif self.RightStop:
            self.grabermotor.set(0)
            print('Not Moving Right')

        elif self.LeftStop:
            self.grabermotor.set(0)
            print('Not Moving Left')

        elif self.LeftStart:
            self.grabermotor.set(LeftSpeed)
            print('Moving Left')

        elif donePressed:
            self.encoder.setPosition(0)
            self.grabermotor.set_position(0)
            print('Home.')



        


    def periodic(self):

        if self.state == 'teleop': 
             
            APressed = self.controller.getAButtonPressed()
            BPressed = self.controller.getBButtonPressed()
            go_downPressed = self.controller.getYButtonPressed()
            Go_homePressed = self.controller.getLeftBumperPressed()
            lv3flipPressed = self.controller.getRightBumperPressed()
            homePressed = self.controller.getXButtonPressed()

            if APressed:
                self.position_home()
                
            elif Go_homePressed:
                self.position_out()

            elif go_downPressed:
                self.go_down()

            elif lv3flipPressed:
                self.lv3flip()

            elif BPressed:
                self.lv1flip()

            elif homePressed:
                self.state = 'test'

        if self.state == 'auto':
            pass

        if self.state == 'test':
            self.testexacute()
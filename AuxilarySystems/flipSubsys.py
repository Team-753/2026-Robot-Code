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
        self.grabermotor=phoenix6.hardware.TalonFX(auxiliaryConfig.flipGrabMotorID)
        self.encoder = self.armmotor.getEncoder()
        self.armmotorController = self.armmotor.getClosedLoopController()
        self.brake = controls.NeutralOut()

        graberconfigs = phoenix6.configs.Slot0Configs()
        graberconfigs.k_s = auxiliaryConfig.graber_k_s_config
        graberconfigs.k_v = auxiliaryConfig.graber_k_v_config
        graberconfigs.k_p = auxiliaryConfig.graber_k_p_config
        graberconfigs.k_i = auxiliaryConfig.graber_k_i_config
        graberconfigs.k_d = auxiliaryConfig.graber_k_d_config
        rampconfig = phoenix6.configs.ClosedLoopRampsConfigs()
        rampconfig.voltage_closed_loop_ramp_period = 1
        grabberFeedbackConfig = phoenix6.configs.FeedbackConfigs()
        grabberFeedbackConfig.sensor_to_mechanism_ratio = auxiliaryConfig.flipkrakenGearRatio
        grabberCurrentConfig = phoenix6.configs.CurrentLimitsConfigs()
        grabberCurrentConfig.stator_current_limit = 120
        grabberCurrentConfig.stator_current_limit_enable = True
  
        arm_k_p_config=rev.SparkMaxConfig() 
        arm_k_p_config.closedLoop.P(0.8)
        arm_k_p_config.closedLoop.I(0.0)
        arm_k_p_config.closedLoop.D(0.2)

        self.armmotor.configure(arm_k_p_config,rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.grabermotor.configurator.apply(graberconfigs)
        self.grabermotor.configurator.apply(rampconfig)
        self.grabermotor.configurator.apply(grabberFeedbackConfig)
        self.grabermotor.configurator.apply(grabberCurrentConfig)

        # self.armrequests = controls.VelocityVoltage(0).with_slot(0)
        self.graberrequests = controls.PositionVoltage(0).with_slot(0)
        self.controller = wpilib.XboxController(1)

        self.state = 'init'
        self.substate = 'none'
        self.homingstate = 'start'
        self.homepointarm = 'set'
        self.armout = 'start'
        self.lv1flipgo = 'start'
        self.lv3flipgo = 'start'
        self.godownstate = 'start'

        self.graberout = 0
        self.armoutpos = 10
        self.enabled = False
        self.positionset = 1.0
        self.position2set = -1.0
        self.target = 0

        self.lv1flipPressed = False
        self.lv3flipPressed = False
        self.Go_StartPosPressed = False
        self.Go_homePressed = False
        self.Go_DownPressed = False
        
        self.lv1flipPressedPrev = False
        self.lv3flipPressedPrev = False
        self.Go_StartPosPressedPrev = False
        self.Go_homePressedPrev = False
        self.Go_DownPressedPrev = False
        
        self.lv1flipChanged = False
        self.lv3flipChanged = False
        self.Go_StartPosChanged = False
        self.Go_homeChanged = False
        self.Go_DownChanged = False

        self.pov = 0
        self.povprev = 0
        
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

        self.encoder.setPosition(0)
        self.grabermotor.set_position(0)

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
        if self.godownstate == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv0))
            print('moving down - rotary moving to home')
            self.godownstate = 'wait'
        elif self.godownstate == 'wait':
            pos = self.grabermotor.get_position()
            if auxiliaryConfig.graberlv0 < (pos.value + .05) and auxiliaryConfig.graberlv0 > (pos.value - 0.05):
                self.getout = 'done'
                print("move down complete")
                self.enabled = False
                self.godownstate = 'none'
                self.substate = 'none'
       
    def position_home(self):
        if self.homepointarm == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv0))
            print("rotary moving to home")
            self.homepointarm = 'waitforamoment'
        elif self.homepointarm == 'waitforamoment':
           self.pos = self.grabermotor.get_position()
           self.target = auxiliaryConfig.graberlv0
           if self.target < (self.pos.value + .05) and self.target > (self.pos.value - 0.05):
               target_rotations = auxiliaryConfig.flipLinPosIn
               self.armmotorController.setSetpoint(target_rotations, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)
               self.homepointarm = 'done'
               print ("arm moving to home")
        elif self.homepointarm == 'done':   
            self.armpos = self.encoder.getPosition()
            print(self.armpos)
            armtarget = auxiliaryConfig.flipLinPosIn
            if armtarget < (self.armpos + .05) and armtarget > (self.armpos - .05):
                print("arm moved to home")
                self.substate = 'none'
                self.homepointarm = 'none'

    def position_out(self):
        if self.armout == 'start':
            target_rotations = auxiliaryConfig.flipLinPosOut
            self.armmotorController.setSetpoint(target_rotations, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot(0))
            print("arm moving to start pos")
            self.armout = 'wait'
        elif self.armout == 'wait':
            armpos= self.encoder.getPosition()
            armtarget = auxiliaryConfig.flipLinPosOut
            if armtarget < (armpos + .05) and armtarget > (armpos - 0.05):
                self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv0).with_feed_forward(0))
                print("grabber moving to start pos")
                self.armout = 'waitforgrabber'
        elif self.armout == 'waitforgrabber':
            grabbertarget = auxiliaryConfig.graberlv0
            graberpos = self.grabermotor.get_position()
            if grabbertarget < (graberpos.value + .05) and grabbertarget > (graberpos.value - 0.05):
                print("arm and grabber at start pos. Ready for climb.")
                self.enabled = False
                self.substate = 'none'
                self.armout == 'none'
  
    def lv1flip(self):
        if self.lv1flipgo == 'start':
            self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv1).with_feed_forward(0))
            self.lv1flipgo = 'wait'
            print('flipping to lv1')
        elif self.lv1flipgo == 'wait':
            targetpos = auxiliaryConfig.graberlv1
            pos = self.grabermotor.get_position()
            if targetpos < (pos.value + .05) and targetpos > (pos.value - 0.05):
                self.grabermotor.set_control(self.brake)
                print("reached lv1")
                self.enabled = False
                self.lv1flipgo = 'none'
                self.substate = 'none'

    def lv3flip(self):
        newarmpos = self.encoder.getPosition()
        targetpos = auxiliaryConfig.flipLinPosOut
        if targetpos < (newarmpos+ 25) and targetpos > (newarmpos - 25):
            if self.lv3flipgo == 'start':
                self.grabermotor.set_control(self.graberrequests.with_position(auxiliaryConfig.graberlv3).with_feed_forward(0))
                self.lv3flipgo = 'wait'
                print('flipping to lv3')
            elif self.lv3flipgo == 'wait':
                targetpos = auxiliaryConfig.graberlv3
                pos = self.grabermotor.get_position()
                # print(pos)
                if targetpos < (pos.value + .01) and targetpos > (pos.value - 0.01):
                    self.grabermotor.set_control(self.brake)
                    print("reached lv3")
                    self.enabled = False
                    self.lv3flipgo = 'none'
                    self.substate = 'none'
        else:   
            self.enabled = False
            self.lv3flipgo = 'none'
            self.substate = 'none'
            print('ignoring flip')
    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'

    def setToIdle(self):
        self.state = 'idle'

    def testInit(self):
        self.state = 'test'

    def autoGoHomePos(self):
        self.Go_homePressed = True    
    
    def autoGoStartPos(self):
        self.Go_StartPosPressed = True    

    def autoClimbLv1(self):
        self.lv1flipPressed = True

    def autoClimbLv3(self):
        self.lv3flipPressed = True

    def autoGoDown(self):
        self.Go_DownPressed = True

    def isMoveDone(self):
        return self.substate == 'none'

    def testexacute(self):
        self.donePressed = self.controller.getRawButtonPressed(auxiliaryConfig.FLipHomeDoneButtonINdex)
        # INSpeed = 1
        # LeftSpeed = 1

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
            self.armmotor.set(-auxiliaryConfig.armhomespeed)
            print('flip - Moving in')

        elif self.INStop:
            self.armmotor.set(0)
            print('flip - not Moving in')

        elif self.OutStop:
            self.armmotor.set(0)
            print('flip - not Moving out')

        elif self.OutStart:
            self.armmotor.set(auxiliaryConfig.armhomespeed)
            print('flip - Moving out')

        elif self.RightStart:
            self.grabermotor.set(-auxiliaryConfig.graberhomespeed)
            print('flip - Moving Right')

        elif self.RightStop:
            self.grabermotor.set(0)
            print('flip - Not Moving Right')

        elif self.LeftStop:
            self.grabermotor.set(0)
            print('flip - Not Moving Left')

        elif self.LeftStart:
            self.grabermotor.set(auxiliaryConfig.graberhomespeed)
            print('flip - Moving Left')

        elif self.donePressed:
            self.encoder.setPosition(0)
            self.grabermotor.set_position(0)
            print('flip - Home saved')

    def periodic(self):

        if self.state == 'teleop': 
            # gather button inputs
            # self.lv1flipPressed = self.controller.getRawButtonPressed(auxiliaryConfig.Flip_Climb_Lv1_BtnIdx)
            # self.lv3flipPressed = self.controller.getRawButtonPressed(auxiliaryConfig.Flip_Climb_Lv3_BtnIdx)
            # self.Go_StartPosPressed = self.controller.getRawButtonPressed(auxiliaryConfig.Flip_Go_Start_BtnIdx)
            # self.Go_homePressed = self.controller.getRawButtonPressed(auxiliaryConfig.Flip_Go_Home_BtnIdx)
            # self.Go_DownPressed = self.controller.getRawButtonPressed(auxiliaryConfig.flip_Go_Down_BtnIdx)

            self.Go_homePressedPrev = self.Go_homePressed
            self.Go_StartPosPressedPrev = self.Go_StartPosPressed
            self.lv1flipPressedPrev = self.lv1flipPressed
            self.lv3flipPressedPrev = self.lv3flipPressed


            self.povprev = self.pov
            self.pov = self.controller.getPOV()
            
            if self.pov != -1 :
                if self.pov > 180 - 5 and self.pov < 180 + 5 :
                    self.Go_homePressed = True
                elif self.pov > 270 - 5 and self.pov < 270 + 5 :
                    self.Go_StartPosPressed = True
                elif self.pov > 90 - 5 and self.pov < 90 + 5:
                    self.lv1flipPressed = True
                elif self.pov > 0 - 5 and self.pov < 0 + 5:
                    self.lv3flipPressed = True
            else:
                self.lv1flipPressed = False
                self.lv3flipPressed = False
                self.Go_StartPosPressed = False
                self.Go_homePressed = False
                self.Go_DownPressed = False

            self.Go_homeChanged = self.Go_homePressedPrev == False and self.Go_homePressed == True
            self.Go_StartPosChanged = self.Go_StartPosPressedPrev == False and self.Go_StartPosPressed == True
            self.lv1flipChanged = self.lv1flipPressedPrev == False and self.lv1flipPressed == True
            self.lv3flipChanged = self.lv3flipPressedPrev == False and self.lv3flipPressed == True
            

            #excute state
            self.execute()

        elif self.state == 'auto':
            # inputs captured from the auto function calls
            # execute state
            self.execute()
            # revert all pressed states back when in auto
            self.lv1flipPressed = False
            self.lv3flipPressed = False
            self.Go_StartPosPressed = False
            self.Go_homePressed = False
            self.Go_DownPressed = False
            self.lv1flipChanged = False
            self.lv3flipChanged = False
            self.Go_StartPosChanged = False
            self.Go_homeChanged = False
            self.Go_DownChanged = False

        elif self.state == 'test':
            self.testexacute()
        
        else :
            self.substate = 'none'
            # make sure everything is reset here
            self.lv1flipPressed = False
            self.lv3flipPressed = False
            self.Go_StartPosPressed = False
            self.Go_homePressed = False
            self.Go_DownPressed = False
            self.lv1flipChanged = False
            self.lv3flipChanged = False
            self.Go_StartPosChanged = False
            self.Go_homeChanged = False
            self.Go_DownChanged = False

    def execute(self):

        if self.Go_homeChanged:
            self.homepointarm = 'start'
            self.substate = 'goHome'

        elif self.Go_StartPosChanged:
            self.armout = 'start'
            self.substate = 'goStart'

        elif self.Go_DownChanged: # rotate robot from flipped state back down to the ground
            self.godownstate = 'start'
            self.substate = 'goDown'

        elif self.lv3flipChanged:
            self.lv3flipgo = 'start'
            self.substate = 'lv3flip'

        elif self.lv1flipChanged:
            self.lv1flipgo = 'start'
            self.substate = 'lv1flip'

        # if we are in a substate, execute it
        if self.substate == 'goStart':
            self.position_out()
        elif self.substate == 'goHome':
            self.position_home()
        elif self.substate == 'lv1flip':
            self.lv1flip()
        elif self.substate == 'lv3flip':
            self.lv3flip()
        elif self.substate == 'goDown':
            self.go_down()
    def setGrabber(self,speed):
        print("grab",speed)
        if speed==0.016:
            self.grabermotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.grabermotor.set(speed)
    def setTrack(self,speed):
        print('track',speed)
        self.armmotor.set(speed)

class flipGrabberCommand(commands2.Command):
    def __init__(self,flipsub:flipsubsys,speed):
        super().__init__()
        self.flip=flipsub
        self.speed=speed
    def execute(self):
        self.flip.setGrabber(self.speed)
    def end(self,interrupted):
        self.flip.setGrabber(0)

class flipTrackCommand(commands2.Command):
    def __init__(self,flipsub:flipsubsys,speed):
        super().__init__()
        self.flip=flipsub
        self.speed=speed
    def execute(self):
        print("track")
        self.flip.setTrack(self.speed)
    def end(self,interupted):
        self.flip.setTrack(0.016)
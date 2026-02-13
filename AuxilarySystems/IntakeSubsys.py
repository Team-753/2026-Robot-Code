import commands2
import wpilib
import phoenix6
from phoenix6 import configs, controls
import rev
from AuxilarySystems import auxiliaryConfig

class intakeSubsys(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.timer = wpilib.Timer()
        self.state = 'init'
        self.spin = phoenix6.hardware.TalonFX(auxiliaryConfig.intakeSpinMotorID)
        self.updown = rev.SparkMax(auxiliaryConfig.intakeUpDownMotorID,rev.SparkMax.MotorType.kBrushless)
        self.updownEncoder = self.updown.getEncoder()
        self.updownController = self.updown.getClosedLoopController()
        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.11
        big_config.k_i = 0
        big_config.k_d = 0
        big_config.k_s = 0.1
        big_config.k_v = 0.12
        
        self.updownConfig = rev.SparkMaxConfig() 
        self.updownConfig.closedLoop.P(0.1, 0)
        self.updownConfig.closedLoop.I(0.0, 0)
        self.updownConfig.closedLoop.D(0.0, 0)

        self.spin.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controller =  wpilib.XboxController(0) #wpilib.Joystick(0)
        self.YPressed = False
        self.prevVal = False
        self.YChanged = False
        self.APressed = False
        self.prevVal2 = False
        self.AChanged = False
        self.whatDIR = True
        self.timer.reset()
        self.timer.start()
        self.targetVelocity = 0
        self.spinToggle = False
        self.inRange = False

        self.AbsEncoder = wpilib.DutyCycleEncoder(0)
        self.AbsEncoder.get()
        self.updown.set_position(self.convertRotations(self.AbsEncoder.get()))

    def convertMotorRotations(self, absValue):
        return absValue * 50 + auxiliaryConfig.intakeUpDownEncoderOffset # this math is to be determined later
        
    def convertAbsRotations(self, absValue):
        return (absValue + auxiliaryConfig.intakeUpDownEncoderOffset) * 360

    def teleopInit(self):
        self.state = 'teleop'

    def periodic(self):

        print (self.AbsEncoder.get())

        if self.state == 'teleop':
            self.executeState()

    def executeState(self):
        
        self.prevVal = self.YPressed
        #self.YPressed = self.controller.getRawButton(auxiliaryConfig.shooterEnableBtnIdx)
        self.YPressed = self.controller.getYButton()
        self.YChanged = self.prevVal == False and self.YPressed == True

        self.prevVal2 = self.APressed
        #self.APressed = self.controller.getRawButton(auxiliaryConfig.shooterEnableBtnIdx)
        self.APressed = self.controller.getAButton()
        self.AChanged = self.prevVal2 == False and self.APressed == True

        self.prevVal3 = self.inRange
        AbsEncoderConverted = self.convertAbsRotations(self.AbsEncoder.get())
        self.inRange = (AbsEncoderConverted < auxiliaryConfig.intakeDownPosition + 15) and (AbsEncoderConverted > auxiliaryConfig.intakeDownPosition - 5)
        self.enteredRange = self.prevVal3 == False and self.inRange == True

        if not self.inRange:
            self.spin.set_control(self.request.with_velocity(0))
            self.spinToggle = False

        if self.state == 'teleop':

            if self.YChanged and self.inRange:
                
                self.spinToggle = not self.spinToggle
                if self.spinToggle == True:
                    self.spin.set_control(self.request.with_velocity(auxiliaryConfig.intakeSpinnerSpeed))
                else:
                    self.spin.set_control(self.request.with_velocity(0))

            if self.AChanged:
                if self.whatDIR == False:
                    targetDirection = ((auxiliaryConfig.intakeUpPosition / 360) * 50)
                    
                    self.whatDIR = True
                else:
                    targetDirection = ((auxiliaryConfig.intakeDownPosition / 360) * 50)
                    self.whatDIR = False
                # call for position move
                self.updownController.setSetpoint(targetDirection, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot(0))
                print (f'moving to {targetDirection / 50}')
            
            

            if self.timer.get() > .99 :
                print (self.AbsEncoder.get())
                self.timer.reset()
                self.timer.start()            

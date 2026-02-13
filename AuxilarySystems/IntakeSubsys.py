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

        self.AbsEncoder = wpilib.DutyCycleEncoder(0)
        self.AbsEncoder.get()
        
        

    def teleopInit(self):
        self.state = 'teleop'

    def periodic(self):

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

        if self.state == 'teleop':

            if self.YChanged:
                
                self.spinToggle = not self.spinToggle

            if self.AChanged:
                if self.whatDIR == False:
                    targetDirection = ((auxiliaryConfig.intakeUpPosition / 360) * 50)
                    self.whatDIR = True
                else:
                    targetDirection = ((auxiliaryConfig.intakeDownPosition / 360) * 50)
                    self.whatDIR = False
                # call for position move
                self.updownController.setSetpoint(targetDirection, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot(0))

            if self.timer.get() > .99 :
                print(f'current intake velocity:{self.spin.get_velocity().value}, target velocity {self.targetVelocity}')
                self.timer.reset()
                self.timer.start()            

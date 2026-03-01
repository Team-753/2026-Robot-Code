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

        # # alternate motor for updown - for testing only
        # self.updownAlt = phoenix6.hardware.TalonFX(8)
        # self.updownAltConfig= phoenix6.configs.Slot1Configs()
        # self.updownAltConfig.k_p = 0.1
        # self.updownAltConfig.k_d = 0.1
        # self.updownAltConfig.k_i = 0
        # #self.updownAltConfig.k_s = 0.1
        # #self.updownAltConfig.k_v = 0.1
        # self.updownAltGeneral = phoenix6.configs.config_groups.ClosedLoopGeneralConfigs()
        # self.updownAltGeneral.continuous_wrap = False
        # self.updownAltFeedback = phoenix6.configs.config_groups.FeedbackConfigs()
        # self.updownAltFeedback.sensor_to_mechanism_ratio = 1
        # self.updownAlt.configurator.apply(self.updownAltConfig)
        # self.updownAlt.configurator.apply(self.updownAltGeneral)
        # self.updownAlt.configurator.apply(self.updownAltFeedback)
        # self.positionRequest = controls.PositionVoltage(0).with_slot(1) # vel = 5rps
        # self.updownAlt.set_position(0)

        big_config = phoenix6.configs.Slot0Configs()
        big_config.k_p = 0.11
        big_config.k_i = 0
        big_config.k_d = 0
        big_config.k_s = 0.1
        big_config.k_v = 0.12
        
        self.updownConfig = rev.SparkMaxConfig()
        self.updownConfig.closedLoop.P(0.5)
        self.updownConfig.closedLoop.I(0.00001)
        self.updownConfig.closedLoop.D(0.0)
        self.updownConfig.closedLoop.IMaxAccum(0.2)
        self.updownConfig.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)
        self.updown.configure(self.updownConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.spin.configurator.apply(big_config)
        self.request = controls.VelocityVoltage(0).with_slot(0)
        self.controllerPort = auxiliaryConfig.auxControllerSlot
        self.controller = wpilib.XboxController(self.controllerPort) #wpilib.Joystick(0)
        self._warnedMissingController = False
        self.YPressed = False
        self.prevVal = False
        self.YChanged = False
        self.APressed = False
        self.prevVal2 = False
        self.AChanged = False
        # Assume intake starts in the down position so first toggle goes up.
        self.intakeIsDown = True
        self.timer.reset()
        self.timer.start()
        self.targetVelocity = 0
        self.spinToggle = False
        self.inRange = False
        

        self.AbsEncoder = wpilib.DutyCycleEncoder(0)

    def convertMotorRotations(self, absValue):
        return absValue * 50 # this math is to be determined later
        
    def convertAbsRotations(self, absValue):
        return absValue + auxiliaryConfig.intakeUpDownEncoderOffset

    def teleopInit(self):
        self.state = 'teleop'

    def autoInit(self):
        self.state = 'auto'
        
    def setToIdle(self):
        self.state = 'idle'

    def periodic(self):

        #print (self.AbsEncoder.get())

        if self.state == 'teleop':
            self.executeState()

    def executeState(self):
        
        self.prevVal = self.YPressed
        self.YPressed = self._getRawButtonSafe(auxiliaryConfig.intakeSpinEnableBtnIdx)
        self.YChanged = self.prevVal == False and self.YPressed == True

        self.prevVal2 = self.APressed
        self.APressed = self._getRawButtonSafe(auxiliaryConfig.intakeUpdownToggleBtnIdx)
        self.AChanged = self.prevVal2 == False and self.APressed == True

        self.prevVal3 = self.inRange
        AbsEncoderConverted = 0#self.convertAbsRotations(self.AbsEncoder.get())
        self.inRange = (AbsEncoderConverted < auxiliaryConfig.intakeDownPosition/360 + 15/360) and (AbsEncoderConverted > auxiliaryConfig.intakeDownPosition/360 - 5/360)
        self.enteredRange = self.prevVal3 == False and self.inRange == True

        if not self.inRange:
            self.spin.set_control(self.request.with_velocity(0))
            self.spinToggle = False

        if self.state == 'teleop':

            if self.YChanged and self.inRange:
                
                self.spinToggle = not self.spinToggle
                if self.spinToggle == True:
                    self.spin.set(-auxiliaryConfig.intakeSpinnerSpeed)
                    print('intake start spinning')
                else:
                    self.spin.set(0)
                    print('intake stop spinning')

            if self.AChanged:
                if self.intakeIsDown:
                    targetDirection = ((auxiliaryConfig.intakeUpPosition / 360) * auxiliaryConfig.intakeupdowngearratio)
                    self.intakeIsDown = False
                    targetLabel = "up"
                else:
                    targetDirection = ((auxiliaryConfig.intakeDownPosition / 360) * auxiliaryConfig.intakeupdowngearratio)
                    self.intakeIsDown = True
                    targetLabel = "down"

                # call for position move
                err = self.updownController.setSetpoint(
                    targetDirection,
                    rev.SparkMax.ControlType.kPosition,
                    rev.ClosedLoopSlot.kSlot0,
                )
                # also call it from the test updown
                # self.updownAlt.set_control(self.positionRequest.with_position(targetDirection))
                if err == rev.REVLibError.kOk:
                    print(f'intake moving {targetLabel} to {targetDirection}')
                else:
                    print(f'intake move {targetLabel} failed: {err}')
            
            

            if self.timer.get() > .99 :
                #print (f'AbsEncoderConverted {AbsEncoderConverted}')
                self.timer.reset()
                self.timer.start()

    def _getRawButtonSafe(self, buttonIdx: int) -> bool:
        buttonCount = wpilib.DriverStation.getStickButtonCount(self.controllerPort)
        if buttonCount < buttonIdx:
            if not self._warnedMissingController:
                print(
                    f"Aux controller on USB {self.controllerPort} has {buttonCount} buttons; "
                    f"cannot read button {buttonIdx}"
                )
                self._warnedMissingController = True
            return False
        self._warnedMissingController = False
        return self.controller.getRawButton(buttonIdx)

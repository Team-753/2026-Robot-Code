import rev,wpimath,wpilib
import commands2
import wpimath.controller
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.topMotor = rev.SparkMax(RobotConfig.coralCannon.TopMotorID, rev.SparkMax.MotorType.kBrushed)
        self.bottomMotor = rev.SparkMax(RobotConfig.coralCannon.BottomMotorID, rev.SparkMax.MotorType.kBrushed)
        config = rev.SparkMaxConfig()
        config.follow(RobotConfig.coralCannon.TopMotorID, True)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.pivotMotor.getAbsoluteEncoder()
        self.pivotMotor.IdleMode(0)

        pivotConfig = rev.SparkMaxConfig()
        self.pid = self.pivotMotor.getClosedLoopController()
        pivotConfig.closedLoop.pid(2.0, 0.000, 0.001, slot=rev.ClosedLoopSlot.kSlot0)
        pivotConfig.closedLoop.IMaxAccum(0.1,slot=rev.ClosedLoopSlot.kSlot0)
        pivotConfig.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)
        pivotConfig.inverted(True)
        self.pivotMotor.configure(pivotConfig, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        
    def place(self):
        print("cannon is placing")
        self.topMotor.set(-0.47)

    def slowPlace(self):
        self.topMotor.set(-0.1)
        
    def intake(self):
        self.topMotor.set(1) #full speed

    def topAlgaeRemoval(self):
        self.topMotor.set(-1)
        print("top algae go away")

    def bottomAlgaeRemoval(self):
        self.topMotor.set(1)
        print("bottom algae go away")

    def stop(self):
        self.topMotor.set(0)
    
    def idle(self):
        self.topMotor.set(0)
        print("current cannon postition: " + str(self.encoder.getPosition()))


    def spinup(self):
        self.pivotMotor.set(0.2) # slow speed, because otherwise things break
        print("current cannon postition: " + str(self.encoder.getPosition()))

    def spindown(self):
        self.pivotMotor.set(-0.2)

    def angleStop(self):
        self.pivotMotor.set(0.016)
        
    def angleIdle(self):
        self.pivotMotor.set(0.016)

    def goToPos(self,desPos):  #makes the cannon go to the desired position
        self.desiredPos = desPos
        self.pid.setReference(self.desiredPos, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, 0)
        print("CANNONING",desPos)
        
    def periodic(self):
        #print("cannon position: " + str(self.encoder.getPosition()))
        pass

class place(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        print("place command is running")

    def execute(self):
        self.cannon.place()
        print("placing")

    def end(self, interrupted: bool):
        self.cannon.idle()
        #return super().end(interrupted)

class intake(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.intake()
        print("intaking")
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.idle()
        
        #return super().end(interrupted)
        #add idle for position
        
class PivotUp(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.spinup()
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.angleIdle()
        
        
class PivotDown(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        #print("intake command is running")

    def execute(self):
        self.cannon.spindown()
        #add intake position function

    def end(self, interrupted: bool):
        self.cannon.angleIdle()
        
        
class DefaultPivotCommand(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.ManualControl(self.cannon.GetJoystickInput())

    def end(self, interrupted):
        self.cannon.angleStop()
        return super().end(interrupted)
    
    
class cannonToPosition(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, desPos):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        self.desiredPosition = desPos

    def execute(self):
        self.cannon.goToPos(self.desiredPosition)
        print("stuff")
        return super().execute()
    
    def end(self, interrupted: bool):
        self.cannon.angleIdle()

class TopAlgaeRemoval(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.topAlgaeRemoval()
        print("removing algae")
        self.cannon.goToPos(0.2)

    def end(self, interrupted: bool):
        self.cannon.idle()
        self.cannon.angleStop()


class BottomAlgaeRemoval(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem

    def execute(self):
        self.cannon.bottomAlgaeRemoval()
        print("removing algae")
        self.cannon.goToPos(0.13)

    def end(self, interrupted: bool):
        self.cannon.idle()
        self.cannon.angleStop()

class AutoCannonPosition(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, desPos, stopTime):
        super().__init__()
        self.addRequirements(cannonSubsystem)
        self.cannon = cannonSubsystem
        self.timer = wpilib.Timer()
        self.endTime = stopTime
        self.desiredPose = desPos

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        
    def execute(self):
        self.cannon.goToPos(self.desiredPose)

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        #self.timer.reset()

    def end(self, interrupted: bool):
        self.timer.stop()
        

class AutoPlace(commands2.Command):
    def __init__(self, kcannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = kcannonSubsystem
        self.timer = wpilib.Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.cannon.place()

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        #self.timer.reset()

    def end(self, interrupted: bool):
        self.timer.stop()
        self.cannon.idle()


class AutoIntake(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = cannonSubsystem
        self.timer = wpilib.Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.cannon.intake()

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        #self.timer.reset()

    def end(self, interrupted: bool):
        self.timer.stop()


class AutoTroughPlace(commands2.Command):
    def __init__(self, cannonSubsystem: CannonSubsystem, stopTime):
        super().__init__()
        self.addRequirements(CannonSubsystem)
        self.cannon = CannonSubsystem
        self.timer = wpilib.Timer()
        self.endTime = stopTime

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.cannon.slowPlace()

    def isFinished(self):
        if self.timer.hasElapsed(self.endTime):
            return True
        #self.timer.reset()

    def end(self, interrupted: bool):
        
        self.timer.stop()


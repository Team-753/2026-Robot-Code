import rev,wpilib,commands2,wpimath,math
import wpimath.controller
import AuxilarySystems.auxiliaryConfig
def constrain(var,min,max):
    if var>max:
        return max
    elif var<min:
        return min
    else:
        return var
    
class elevatorSubSystem(commands2.Subsystem):
    def __init__(self):
        '''Chris, I think the way to get this to work is to use rev's built in PID controller
        Currently we are setting the desired position, but we might also be passing it as a speed.
        That will mess stuff up. the built in pid controller has a method that will make it a lot easier
        You can use different values in your PIDs fairly easily by using multiple slots - Sanika'''
        #gets ID
        lMotorID=AuxilarySystems.auxiliaryConfig.elevatorMotorID[0]
        rMotorID=AuxilarySystems.auxiliaryConfig.elevatorMotorID[1]
        #sets motor based off motorid
        self.lMotor=rev.SparkMax(lMotorID,rev.SparkMax.MotorType.kBrushless)
        self.rMotor=rev.SparkMax(rMotorID,rev.SparkMax.MotorType.kBrushless)
        #read it
        self.pid=self.lMotor.getClosedLoopController()
        self.encoder=self.lMotor.getEncoder()
        self.absEncoder=self.lMotor.getAbsoluteEncoder()
        configL=rev.SparkMaxConfig()       
        configR=rev.SparkMaxConfig()
        configR.follow(lMotorID)
        configL.closedLoop.pid(0.05,0.0000001,0.01,slot=rev.ClosedLoopSlot.kSlot0)
        configL.closedLoop.IMaxAccum(0.1,slot=rev.ClosedLoopSlot.kSlot0)
        self.encoder.setPosition(0)
        self.rMotor.configure(configR, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        self.lMotor.configure(configL, rev.ResetMode.kNoResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        self.lMotor.IdleMode(0)
        self.rMotor.IdleMode(0)
        self.desiredPos = 0
    def checkBottom(self):
        if self.encoder.getPosition()<0.7:
            self.lMotor.IdleMode(0)
            self.lMotor.set(0)
            return True
        else:
            return False
            #print("MOTOR IDLE")
    def setPosition(self,desiredPos):
        self.desiredPos=desiredPos
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0,1.2)
        print("ELEVATING")
    def goToZero(self):
        if self.encoder.getPosition()<0.7:
                self.lMotor.set(0)
                #print("MOTOR IDLE")
        elif self.encoder.getPosition()<4 and abs(self.encoder.getVelocity())<150:
            self.lMotor.set(0.018)
            #print("AT POS")
        else:
            self.pid.setReference(4,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def goToMax(self):
       # if self.encoder.getPosition()>23:
            #    self.lMotor.set(0.05)
                #print("MOTOR IDLE")
        if self.encoder.getPosition()>21 and abs(self.encoder.getVelocity())<150:
            self.lMotor.set(0.1)
            print("AT POS")
        else:
            self.pid.setReference(23,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def goUp(self):
        #if not self.encoder.getPosition()>26:
        self.lMotor.set(0.15)
        print(self.encoder.getPosition())
    def goDown(self):
        self.lMotor.set(0.017)
        print(self.encoder.getPosition())
    def holdPos(self):
        self.desiredPos=self.encoder.getPosition()
        self.pid.setReference(self.desiredPos,rev.SparkMax.ControlType.kPosition,rev.ClosedLoopSlot.kSlot0)
    def constantUp(self):
        self.lMotor.IdleMode(0)
        self.lMotor.set(0.06)
    def getPosError(self):
        return (float(self.desiredPos)-float(self.encoder.getPosition()))


class elevatorUp(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goUp()
        #print("Going down")
    def end(self, interrupted):
        #self.eSub.holdPos()
        self.eSub.constantUp()
        self.eSub.checkBottom()
        pass
    
    
class elevatorDown(commands2.Command):
    def __init__(self, kElevSub:elevatorSubSystem):
        super().__init__()
        self.addRequirements(kElevSub)
        self.eSub=kElevSub
    def execute(self):
        self.eSub.goDown()
        
        #print("Going up")
        
    def end(self, interrupted):
        #self.eSub.holdPos()
        self.eSub.constantUp()
        self.eSub.checkBottom()
        pass

'''class elevatorToPos(commands2.Command):
    def __init__(self, elevatorSubSystem: elevatorSubSystem, desPos):
        super().__init__()
        self.addRequirements(elevatorSubSystem)
        self.elevator = elevatorSubSystem
        self.desiredPos=desPos
    def execute(self):
        if self.desiredPos==0:
            self.elevator.goToZero()
            self.elevator.checkBottom()
        else:
            self.elevator.setPosition(self.desiredPos)'''

class elevatorToPos(commands2.Command):
    def __init__(self, elevatorSubSystem: elevatorSubSystem, desPos):
        super().__init__()
        self.addRequirements(elevatorSubSystem)
        self.elevator = elevatorSubSystem
        self.desiredPos=desPos
    def execute(self):
        wpilib.SmartDashboard.putBoolean("elevator going", True)
        if self.desiredPos==0:
            self.elevator.goToZero()
            self.elevator.checkBottom()
        elif self.desiredPos == 22:
            self.elevator.goToMax()
        else:
            self.elevator.setPosition(self.desiredPos)
    def isFinished(self):
        if float(self.elevator.getPosError())<0.2 and self.desiredPos!=0:
            print("elevatorcommandfinished")
            wpilib.SmartDashboard.putBoolean("elevator going", False)
            return True
        elif self.desiredPos==0 and self.elevator.checkBottom():
            wpilib.SmartDashboard.putBoolean("elevator going", False)
            return True
        else:
            return False
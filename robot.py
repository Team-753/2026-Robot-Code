
import wpilib
import commands2
from robotContainer import robotContainer
class myRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.rContainer=robotContainer()
    def robotPeriodic(self):
        self.rContainer.ensureButtonBindings()
        self.rContainer.syncDashboardControls()
        self.rContainer.updateTrajectoryPreview()
    def teleopInit(self):
        self.rContainer.teleopInit()
    def autonomousInit(self):
        self.rContainer.autoInit()
    def autonomousPeriodic(self):
        self.rContainer.autoPeriodic()
    def disabledInit(self):
        self.rContainer.disabledInit()
    def disabledPeriodic(self):
        pass
    def testInit(self):
        self.rContainer.testInit()
    def simulationPeriodic(self):
        pass

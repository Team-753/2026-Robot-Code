
import wpilib
import commands2
from robotContainer import robotContainer
class myRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.rContainer=robotContainer()
    def teleopInit(self):
        self.rContainer.teleopInit()
    def autonomousInit(self):
        self.rContainer.autoInit()
    def disabledInit(self):
        self.rContainer.disabledInit()

import commands2

from robotContainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def __init__(self, period=.02):
        super().__init__(period)
        self.robot_container = RobotContainer()


    
    def robotInit(self):
        pass
    
    def autonomousInit(self):
        """Code run at the start of auto. This currently figures out what the robot is doing for auto"""
        pass
    

    def teleopInit(self):
        pass

          

    def disabledInit(self):
        pass
    
    
    

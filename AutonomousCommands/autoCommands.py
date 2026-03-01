import commands2,wpilib
from AuxilarySystems import flipSubsys,IntakeSubsys,shooterSubsys
def setShooting(shooterState):
    if shooterState:
        print("settingShooter"+shooterState)
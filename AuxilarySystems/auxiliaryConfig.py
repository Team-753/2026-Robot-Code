auxController="XboxController" #Options:XboxController
auxControllerSlot=1

#Shooter Subystem Config
shooterIndexMotorID=6 #Rev SparkMAX
#All are Phoenix6 TalonFX
shooterMotorID1=14 # + velocity
shooterMotorID2=15 # + velocity
shooterMotorID3=13 # - velocity
shooterMotorID4=16 # - velocity
shooterEnableBtnIdx=3 # AXIS index for enable button
shooterVelocityUpBtnIdx=4 # raw button index for velocity up increment
shooterVelocityDownBtnIdx=1 # raw button index for velocity down increment
shooterIndexDutyCycle = 0.8 # fixed duty cycle of the rev indexer

#Indexer Subsystem config
indexerSpeed=0.5 #1 is 100% power
indexerMotorIndexNumberLIKETHEONLYMOTOR = 17
indexerEnableBtnIdx=2

#Intake Subsystem config
intakeUpDownMotorID=9 #?
intakeSpinMotorID=20
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 110 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 0.7 #dutyCycle
intakeupdowngearratio = 1
intakeSpinEnableBtnIdx = 3
intakeUpdownToggleBtnIdx = 5
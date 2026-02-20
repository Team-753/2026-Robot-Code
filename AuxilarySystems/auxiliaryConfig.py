auxController="XboxController" #Options:XboxController
auxControllerSlot=1

#Shooter Subystem Config
shooterIndexMotorID=18 #Rev SparkMAX
#All are Phoenix6 TalonFX
shooterMotorID1=14 # + velocity
shooterMotorID2=15 # + velocity
shooterMotorID3=13 # - velocity
shooterMotorID4=16 # - velocity
shooterEnableBtnIdx=3 # AXIS index for enable button
shooterVelocityUpBtnIdx=4 # raw button index for velocity up increment
shooterVelocityDownBtnIdx=1 # raw button index for velocity down increment
shooterIndexDutyCycle = 0.1 # fixed duty cycle of the rev indexer

#Indexer Subsystem config
indexerSpeed=5 #revolutions per second
indexerMotorIndexNumberLIKETHEONLYMOTOR = 17
indexerEnableBtnIdx=2

#Intake Subsystem config
intakeUpDownMotorID=98
intakeSpinMotorID=5
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
intakeupdowngearratio = 15
intakeSpinEnableBtnIdx = 3 
intakeUpdownToggleBtnIdx = 5
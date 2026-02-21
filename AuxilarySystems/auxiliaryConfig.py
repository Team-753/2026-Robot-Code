auxController="XboxController" #Options:XboxController
auxControllerSlot=1

#Shooter Subystem Config
shooterIndexMotorID=6 #Rev SparkMAX
shooterMotorID1=13 #All are Phoenix6 TalonFX
shooterMotorID2=14
shooterMotorID3=15
shooterMotorID4=16
shooterEnableBtnIdx=3 # joystick raw button index for enable button
shooterVelocityUpBtnIdx=6 # joystick raw button index for velocity up increment
shooterVelocityDownBtnIdx=4 # joystick raw button index for velocity down increment

#Indexer Subsystem config
indexerSpeed=5 #revolutions per second
indexerMotorIndexNumberLIKETHEONLYMOTOR = 17
indexerEnableBtnIdx=2

#Intake Subsystem config
intakeUpDownMotorID=9
intakeSpinMotorID=20
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
intakeupdowngearratio = 15
intakeSpinEnableBtnIdx = 3 
intakeUpdownToggleBtnIdx = 5
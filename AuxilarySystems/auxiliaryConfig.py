auxController="XboxController" #Options:XboxController
auxControllerSlot=1
#Shooter Subystem Config
shooterIndexMotorID=16 #Rev SparkMAX
shooterMotorID1=1 #All are Phoenix6 TalonFX
shooterMotorID2=3
shooterMotorID3=5
shooterMotorID4=7
shooterEnableBtnIdx=3 # joystick raw button index for enable button
shooterVelocityUpBtnIdx=6 # joystick raw button index for velocity up increment
shooterVelocityDownBtnIdx=4 # joystick raw button index for velocity down increment

#Indexer Subsystem config
indexerSpeed=5 #revolutions per second
indexerMotorIndexNumberLIKETHEONLYMOTOR = 5

#Intake Subsystem config
intakeUpDownMotorID=98
intakeSpinMotorID=97
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
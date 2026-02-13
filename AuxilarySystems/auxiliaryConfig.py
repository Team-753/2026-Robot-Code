auxController="XboxController" #Options:XboxController
auxControllerSlot=1
#Shooter Subystem Config
indexMotorID=16 #Rev SparkMAX
shooterMotorID=0 #Phoenix6 TalonFX
shooterEnableBtnIdx=3 # joystick raw button index for enable button
shooterVelocityUpBtnIdx=6 # joystick raw button index for velocity up increment
shooterVelocityDownBtnIdx=4 # joystick raw button index for velocity down increment

#Indexer Subsystem config
indexerSpeed=15 #revolutions per second
indexerMotorIndexNumberLIKETHEONLYMOTOR = 99

#Intake Subsystem config
intakeUpDownMotorID=98
intakeSpinMotorID=97
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
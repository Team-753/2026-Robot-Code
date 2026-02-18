auxController="XboxController" #Options:XboxController
auxControllerSlot=1

#Shooter Subystem Config
shooterIndexMotorID=17 #Rev SparkMAX
#All are Phoenix6 TalonFX
shooterMotorID1=13 # + velocity
shooterMotorID2=15 # + velocity
shooterMotorID3=14 # - velocity
shooterMotorID4=16 # - velocity
shooterEnableBtnIdx=3 # joystick raw button index for enable button
shooterVelocityUpBtnIdx=6 # joystick raw button index for velocity up increment
shooterVelocityDownBtnIdx=4 # joystick raw button index for velocity down increment

#Indexer Subsystem config
indexerSpeed=5 #revolutions per second
indexerMotorIndexNumberLIKETHEONLYMOTOR = 5

#Intake Subsystem config
intakeUpDownMotorID=98
intakeSpinMotorID=5
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
intakeupdowngearratio = 15
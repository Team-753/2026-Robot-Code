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
shooterIndexDutyCycle = 0.7 # fixed duty cycle of the rev indexer
shooterStartupTime = 1.0 #seconds

#Indexer Subsystem config
indexerSpeed=0.5 #1 is 100% power
indexerMotorIndexNumberLIKETHEONLYMOTOR = 17
indexerEnableBtnIdx=3 # AXIS index for enable button - matches the shooter

#Intake Subsystem config
intakeUpDownMotorID=9 #?
intakeSpinMotorID=20
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 0.8 #dutyCycle
intakeupdowngearratio = 1
intakeSpinEnableBtnIdx = 3
intakeUpdownToggleBtnIdx = 5

# flip configs
graberlv1=0.1
graberlv3=0.3

#Gear ratio between the Kraken motor and the rotating part of the climber. 
#For each 506.66666 rotations of the motor, the climber rotates once.
#(506.6666/1) 
krakenGearRatio = 506.66666

graber_k_s_config=0
graber_k_i_config=0
graber_k_p_config=0
graber_k_v_config=0
graber_k_d_config=0
graberhomespeed=0
armhomespeed=0
flipMotorID=19
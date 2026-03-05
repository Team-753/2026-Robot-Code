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
intakeDownPosition = 133  #these are in degrees
intakeUpPosition = 0 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 0.8 #dutyCycle
intakeupdowngearratio = 1
intakeSpinEnableBtnIdx = 3
intakeUpdownToggleBtnIdx = 6

# flip configs
graberlv0=0.0 # position of kraken when not climbing (home)
graberlv1=0.0 #0.1 # position when climbing to lv1
graberlv3=0.0 #0.3 # position when clibing to lv3
flipLinPosIn = 0.0 #-1 # retracted position (home) of rev neo motor
flipLinPosOut = 0 # extended postion (out/start) of rev neo motor

#Gear ratio between the Kraken motor and the rotating part of the climber. 
#For each 506.66666 rotations of the motor, the climber rotates once.
#(506.6666/1) 
flipkrakenGearRatio = 506.66666

graber_k_s_config=0.1
graber_k_i_config=0.1
graber_k_p_config=0.1
graber_k_v_config=0
graber_k_d_config=0
graberhomespeed=0.25
armhomespeed=0.25
flipMotorID=19 # motor id for rev spark max arm motor
flipGrabMotorID=99 # motor id for kraken rotation motor

# button indices for teleop commands
Flip_Go_Home_BtnIdx = 1
Flip_Go_Start_BtnIdx = 2
Flip_Climb_Lv1_BtnIdx = 3
Flip_Climb_Lv3_BtnIdx = 4
flip_Go_Down_BtnIdx = 5

# flip test mode buttons for manually setting zero positions of motors
FLipINButtonINdex=1
FLipOutButtonINdex=4
FLipLeftButtonINdex=3
FLipRightButtonINdex=2
FLipHomeDoneButtonINdex=5
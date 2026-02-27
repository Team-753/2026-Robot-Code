import phoenix6
import rev
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
intakeSpinMotorID=5
intakeDownPosition = 0  #these are in degrees
intakeUpPosition = 120 #these are in degrees
intakeUpDownEncoderOffset = 0 #how far off is the motor that moves the intake up and down(in rotations)
intakeSpinnerSpeed = 30 #in rotations per second
intakeupdowngearratio = 15


# flip configs
graberlv1=0.1
graberlv3=0.3

graber_k_s_config=0
graber_k_i_config=0
graber_k_p_config=0
graber_k_v_config=0
graber_k_d_config=0
graberhomespeed=0
armhomespeed=0
flipMotorID=19
import phoenix6
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
intakeSpeed=15
intakeUpDownMotorID=98
intakeSpinMotorID=97


# flip configs
graberlv1=0.1
graberlv3=0.3
arm_k_p_config=phoenix6.configs.Slot0Configs()
arm_k_p_config.k_p=0
arm_k_p_config.k_i=0
arm_k_p_config.k_d=0

graber_k_s_config=0
graber_k_i_config=0
graber_k_p_config=0
graber_k_v_config=0
graber_k_d_config=0
graberhomespeed=0
armhomespeed=0
flipMotorID=19
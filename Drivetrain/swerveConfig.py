# ____           _____ _____ _____    _____ ____  _   _ ______ _____ _____ 
#|  _ \   /\    / ____|_   _/ ____|  / ____/ __ \| \ | |  ____|_   _/ ____|
#| |_) | /  \  | (___   | || |      | |   | |  | |  \| | |__    | || |  __ 
#|  _ < / /\ \  \___ \  | || |      | |   | |  | | . ` |  __|   | || | |_ |
#| |_) / ____ \ ____) |_| || |____  | |___| |__| | |\  | |     _| || |__| |
#|____/_/    \_\_____/|_____\_____|  \_____\____/|_| \_|_|    |_____\_____|

driveSpeed=50
driveTurnSpeed=60

driveController="Joystick" #Options:XboxController,Joystick,VKBJoystick #NOTE ADD automatic
driveControllerSlot=0 #USB NUMBER IN DRIVESTATION

# Button to hold for targeting (raw button number for HID)
targetingButton = 1

# Starting robot pose (meters, degrees) in WPILib blue-origin field coords
startPoseX = 2
startPoseY = 4
startPoseDeg = 0.0





#ROTATION OFFSETS FOR SWERVE ROTATION SENSORS 
    #HOW DO I GET THESE NUMBERS?
    #I MANUALLY SET THE SWERVES TO THE FORWARD POSITION AND THEN PRINTED OUT THE OFFSETS OF EACH ENCODER AND RECORDED THEM
    #0.071533203125,-0.396728515625,0.16455078125,0.45654296875,
offsetList=[-0.40771484375,0.387939453125,0.4208984375,0.293701171875] ##ONLY APPLIES IF USEING WPI ENCODERS(NOT CANCODER)
debugOffsets=False
##quack 0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847##
##WIWO 0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847##\
##NEWROBOT -0.40771484375 0.387939453125 0.4208984375 0.293701171875 

swerveEncoderType="canCoder" #canCoder,wpilibEncoder #IF NOT USING AN ENCODER LEAVE swerveEncoderIds BLANK
robotCompassType="pidgeon" #navx,pidgeon #NOTE navx not working, need to get inport working
robotCompassId=0 #FOR PIDGEON ONLY
swerveCanivoreName="whoevernamedthiswascool"#Set to None is no canivore, put a string with the name of the canivore if canivore

#SWERVE CAN IDS #FL,FR,BL,BR
swerveDriveIds=[1,4,7,10]   #1,4,7,10]
swerveTurnIds=[2,5,8,11]  # 3,6,9,12]
swerveEncoderIds=[3,6,9,12]#2,5,8,0] #EXTERNAL ENCODERS, NOT MOTORS

#SWERVE MECHANISM RATIOS
    #HOW TO GET?
    #PRINT OUT MOTOR POSITION AND ROTATE THE MECHANISM ~1FULL ROTATION USE END MOTOR POSITION
    #YOU CAN ALSO GOOGLE IT <-------(THIS IS THE BETTER WAY)
swerveDriveRatio=(6.2)#NOT NECESSARY FOR ROBOT MOVEMENT, ONLY FOR ACCURATE AUTO #quack 8.14 #wiwo 5.68 #
swerveWheelDiameter=(0.1016) #METERS
swerveTurnRatio=(1)  #NECECCARY IF NOT USING A CANCODER #quack 12.8

#MEASURE FROM POINT WHEEL CONTACTS GROUND
swerveBaseWidth=(0.61) #METERS
swerveBaseLength=(0.505) #METERS

#ODOMETRY TRUST Levels 
#0-1 scale that is used in the Kalman filter. 0 is full trust, while 1 is full distrust. Hence, the distrust level
visionDistrustLevel = 0.7,0.7,0.7
wheelDistrustLevel = 0.3,0.3,0.3 

#Limelight Camera Name and Settings 
cameraName3 = ("limelight-jamal")
cameraName3a = ("limelight-bubba")
# ____           _____ _____ _____    _____ ____  _   _ ______ _____ _____ 
#|  _ \   /\    / ____|_   _/ ____|  / ____/ __ \| \ | |  ____|_   _/ ____|
#| |_) | /  \  | (___   | || |      | |   | |  | |  \| | |__    | || |  __ 
#|  _ < / /\ \  \___ \  | || |      | |   | |  | | . ` |  __|   | || | |_ |
#| |_) / ____ \ ____) |_| || |____  | |___| |__| | |\  | |     _| || |__| |
#|____/_/    \_\_____/|_____\_____|  \_____\____/|_| \_|_|    |_____\_____|

driveSpeed=-50
driveTurnSpeed=30

driveController="VKBJoystick" #Options:XboxController,Joystick,VKBJoystick
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
offsetList=[-0.459228515625,0.066650390625,0.230224609375,-0.038818359375] ##ONLY APPLIES IF USEING WPI ENCODERS(NOT CANCODER)
##quack 0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847##
##WIWO 0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847## 
##TESTBOT -0.396728515625,0.071533203125,0.16455078125,0.45654296875 ##
swerveEncoderType="canCoder" #canCoder,wpilibEncoder #IF NOT USING AN ENCODER LEAVE swerveEncoderIds BLANK
robotCompassType="navx" #navx,pidgeon
robotCompassId=13 #FOR PIDGEON ONLY

#SWERVE CAN IDS
swerveDriveIds=[1,4,7,10]   #1,4,7,10]
swerveTurnIds=[3,6,9,12]  # 3,6,9,12]
swerveEncoderIds=[2,5,8,11]#2,5,8,0] #EXTERNAL ENCODERS, NOT MOTORS example:cancoders

#SWERVE MECHANISM RATIOS
    #HOW TO GET?
    #PRINT OUT MOTOR POSITION AND ROTATE THE MECHANISM ~1FULL ROTATION USE END MOTOR POSITION
    #YOU CAN ALSO GOOGLE IT <-------(THIS IS THE BETTER WAY)
swerveDriveRatio=(6.2)#NOT NECESSARY FOR ROBOT MOVEMENT, ONLY FOR ACCURATE AUTO #quack 8.14 #wiwo 5.68 #
swerveWheelDiameter=(0.1016) #METERS
swerveTurnRatio=(1)  #NECECCARY IF NOT USING A CANCODER #quack 12.8

#MEASURE FROM POINT WHEEL CONTACTS GROUND
swerveBaseWidth=(0.64) #METERS
swerveBaseLength=(0.64) #METERS

#ODOMETRY TRUST Levels 
#0-1 scale that is used in the Kalman filter. 0 is full trust, while 1 is full distrust. Hence, the distrust level
visionDistrustLevel = 0.5,0.5,0.5
wheelDistrustLevel = 0.5,0.5,0.5 

#Limelight Camera Name and Settings 
cameraName = ("limelight-jamal")

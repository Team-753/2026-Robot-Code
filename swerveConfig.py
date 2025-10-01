
# ____           _____ _____ _____    _____ ____  _   _ ______ _____ _____ 
#|  _ \   /\    / ____|_   _/ ____|  / ____/ __ \| \ | |  ____|_   _/ ____|
#| |_) | /  \  | (___   | || |      | |   | |  | |  \| | |__    | || |  __ 
#|  _ < / /\ \  \___ \  | || |      | |   | |  | | . ` |  __|   | || | |_ |
#| |_) / ____ \ ____) |_| || |____  | |___| |__| | |\  | |     _| || |__| |
#|____/_/    \_\_____/|_____\_____|  \_____\____/|_| \_|_|    |_____\_____|

driveSpeed=4
driveTurnSpeed=2

driveController="Joystick" #Options:XboxController,Joystick
driveControllerSlot=0





#ROTATION OFFSETS FOR SWERVE ROTATION SENSORS 
    #HOW DO I GET THESE NUMBERS?
    #I MANUALLY SET THE SWERVES TO THE FORWARD POSITION AND THEN PRINTED OUT THE OFFSETS OF EACH ENCODER AND RECORDED THEM
offsetList=[0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847 ] ##quack 0.7083120783296617,0.3599781216554734,0.13246701675334055,0.5696525556151847##
#doesnt work#
swerveEncoderType="canCoder" #canCoder,wpilibEncoder #IF NOT USING AN ENCODER LEAVE swerveEncoderIds BLANK
#
#SWERVE CAN IDS
swerveDriveIds=[1,4,7,10]
swerveTurnIds=[3,6,9,12]
swerveEncoderIds=[2,5,8,11]

#SWERVE MECHANISM RATIOS
    #HOW TO GET?
    #PRINT OUT MOTOR POSITION AND ROTATE THE MECHANISM ~1FULL ROTATION USE END MOTOR POSITION
    #YOU CAN ALSO GOOGLE IT <-------(THIS IS THE BETTER WAY)
swerveDriveRatio=(5.68) #NOT NECESSARY FOR ROBOT MOVEMENT, ONLY FOR ACCURATE AUTO #quack 8.14
swerveTurnRatio=(13.3714)  #NECECCARY IF NOT USING AN ENCODER #quack 12.8
#MEASURE FROM POINT WHEEL CONTACTS GROUND
swerveBaseWidth=(0.64) #METERS
swerveBaseLength=(0.64) #METERS
# Input devices and operator controls
class InputDevices:
    class DriverJoystick:
        usbId = 0

        class Deadbands:
            xAxis = 0.1
            yAxis = 0.1
            rotationAxis = 0.1

        class Mappings:
            brakeButton = 1

    class AuxController:
        usbId = 1

        class Mappings:
            pass


# Swerve drivetrain configuration
class Swerve:
    class Geometry:
        trackWidthMeters = 0.6
        wheelBaseMeters = 0.6

    class Speeds:
        maxLinearSpeedMetersPerSecond = 4.8
        maxAngularSpeedRadiansPerSecond = 4.8

    class ModuleConstants:
        driveGearRatio = 6.2
        turningGearRatio = 0.0
        wheelDiameterMeters = 0.04

    class PoseEstimation:
        stateStdDevs = (1.0, 1.0, 1.0)
        visionStdDevs = (0.0, 0.0, 0.0)

    class IMU:
        # Pigeon2 CAN device ID.
        pigeonId = 0

    class frontLeft:
        moduleName = "Front Left"
        driveMotorId = 7
        turningMotorId = 9
        turningEncoderId = 8
        turningEncoderOffset = 0.0

    class frontRight:
        moduleName = "Front Right"
        driveMotorId = 4
        turningMotorId = 6
        turningEncoderId = 5
        turningEncoderOffset = 0.0

    class rearLeft:
        moduleName = "Rear Left"
        driveMotorId = 10
        turningMotorId = 12
        turningEncoderId = 11
        turningEncoderOffset = 0.0

    class rearRight:
        moduleName = "Rear Right"
        driveMotorId = 1
        turningMotorId = 3
        turningEncoderId = 2
        turningEncoderOffset = 0.0


# Vision
class Vision:
    class Limelight:
        limelightName = "Jamal"

    class ProtonCamera:
        protonCameraName = "Proton Camera Name Placeholder"


# Pneumatics
class Pneumatics:
    pass

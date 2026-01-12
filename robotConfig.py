# Input devices and operator controls
class InputDevices:
    class DriverJoystick:
        usbId = 0

        class Deadbands:
            xAxis = 0.0
            yAxis = 0.0
            rotationAxis = 0.0

        class Mappings:
            brakeButton = 1

    class AuxController:
        usbId = 1

        class Mappings:
            pass


# Swerve drivetrain configuration
class Swerve:
    class Geometry:
        trackWidthMeters = 0.0
        wheelBaseMeters = 0.0

    class Speeds:
        maxLinearSpeedMetersPerSecond = 0.0
        maxAngularSpeedRadiansPerSecond = 0.0

    class ModuleConstants:
        driveGearRatio = 0.0
        turningGearRatio = 0.0
        wheelDiameterMeters = 0.0

    class PoseEstimation:
        stateStdDevs = (0.0, 0.0, 0.0)
        visionStdDevs = (0.0, 0.0, 0.0)

    class frontLeft:
        moduleName = "Front Left"
        driveMotorId = 0
        turningMotorId = 0
        turningEncoderId = 0
        turningEncoderOffset = 0.0

    class frontRight:
        moduleName = "Front Right"
        driveMotorId = 0
        turningMotorId = 0
        turningEncoderId = 0
        turningEncoderOffset = 0.0

    class rearLeft:
        moduleName = "Rear Left"
        driveMotorId = 0
        turningMotorId = 0
        turningEncoderId = 0
        turningEncoderOffset = 0.0

    class rearRight:
        moduleName = "Rear Right"
        driveMotorId = 0
        turningMotorId = 0
        turningEncoderId = 0
        turningEncoderOffset = 0.0


# Vision
class Vision:
    class Limelight:
        limelightName = "Limelight Name Placeholder"

    class ProtonCamera:
        protonCameraName = "Proton Camera Name Placeholder"


# Pneumatics
class Pneumatics:
    pass

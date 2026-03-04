import math

from wpilib import Timer, SmartDashboard
import wpilib
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath import geometry
class LimelightCamera(Subsystem):
    
    def __init__(self, cameraName: str) -> None:
        super().__init__()
        self.cameraName = _fix_name(cameraName)
        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()
        self.yaw=0.0
        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.tv = self.table.getDoubleTopic("tv").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.table.getEntry("imumode_set").setInteger(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.lastAcceptedPose = None
        self.pendingPose = None
        self.pendingLatency = None
        self.pendingCount = 0
        self.maxPoseJumpMeters = 0.75
        self.maxPoseJumpDegrees = 30.0
        self.pendingPoseToleranceMeters = 0.45
        self.pendingPoseToleranceDegrees = 20.0
        self.acceptAfterPersistentCycles = 4

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if not self.heartbeating:
            return False
        return self.tv.get(0.0) == 1.0
        

    def setRobotOrientation(self, yaw) -> None:
        # MegaTag2 expects robot yaw in degrees each loop.
        if isinstance(yaw, geometry.Rotation2d):
            self.yaw = float(yaw.degrees())
        else:
            try:
                self.yaw = float(yaw)
            except (TypeError, ValueError):
                self.yaw = 0.0

        self.table.getEntry("robot_orientation_set").setDoubleArray([self.yaw,0.0,0.0,0.0,0.0,0.0])

    def getPoseData(self,yaw) -> tuple[geometry.Pose2d | None, float | None]:
        """ Returns the *last* calculated robot Pose2D and the pipeline latency, or (None, None) if unavailable """
        self.setRobotOrientation(yaw)
        bot_pose_data = self.table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.5,0,0,0,0,0])
        if len(bot_pose_data) < 7:
            return (None, None)
        pose_2d = geometry.Pose2d(
            geometry.Translation2d(bot_pose_data[0], bot_pose_data[1]),
            geometry.Rotation2d.fromDegrees(self.yaw),
        )
        latency = bot_pose_data[6]
        return self._filterPoseMeasurement(pose_2d, latency)

    def _poseDistanceMeters(self, a: geometry.Pose2d, b: geometry.Pose2d) -> float:
        return math.hypot(a.X() - b.X(), a.Y() - b.Y())

    def _rotationDeltaDegrees(self, a: geometry.Rotation2d, b: geometry.Rotation2d) -> float:
        deltaRadians = math.atan2(math.sin(a.radians() - b.radians()), math.cos(a.radians() - b.radians()))
        return abs(math.degrees(deltaRadians))

    def _clearPendingPose(self) -> None:
        self.pendingPose = None
        self.pendingLatency = None
        self.pendingCount = 0

    def _publishPoseFilterStatus(self, status: str, jumpMeters: float = 0.0, jumpDegrees: float = 0.0) -> None:
        SmartDashboard.putString(f"{self.cameraName} Pose Filter", status)
        SmartDashboard.putNumber(f"{self.cameraName} Pose Jump (m)", jumpMeters)
        SmartDashboard.putNumber(f"{self.cameraName} Pose Jump (deg)", jumpDegrees)
        SmartDashboard.putNumber(f"{self.cameraName} Pose Pending Count", self.pendingCount)

    def _filterPoseMeasurement(
        self,
        pose_2d: geometry.Pose2d,
        latency: float,
    ) -> tuple[geometry.Pose2d | None, float | None]:
        if self.lastAcceptedPose is None:
            self.lastAcceptedPose = pose_2d
            self._clearPendingPose()
            self._publishPoseFilterStatus("accepted-initial")
            return (pose_2d, latency)

        jumpMeters = self._poseDistanceMeters(pose_2d, self.lastAcceptedPose)
        jumpDegrees = self._rotationDeltaDegrees(pose_2d.rotation(), self.lastAcceptedPose.rotation())

        if jumpMeters <= self.maxPoseJumpMeters and jumpDegrees <= self.maxPoseJumpDegrees:
            self.lastAcceptedPose = pose_2d
            self._clearPendingPose()
            self._publishPoseFilterStatus("accepted", jumpMeters, jumpDegrees)
            return (pose_2d, latency)

        if self.pendingPose is not None:
            pendingJumpMeters = self._poseDistanceMeters(pose_2d, self.pendingPose)
            pendingJumpDegrees = self._rotationDeltaDegrees(pose_2d.rotation(), self.pendingPose.rotation())
            if (
                pendingJumpMeters <= self.pendingPoseToleranceMeters
                and pendingJumpDegrees <= self.pendingPoseToleranceDegrees
            ):
                self.pendingCount += 1
            else:
                self.pendingCount = 1
        else:
            self.pendingCount = 1

        self.pendingPose = pose_2d
        self.pendingLatency = latency

        if self.pendingCount >= self.acceptAfterPersistentCycles:
            acceptedPose = self.pendingPose
            acceptedLatency = self.pendingLatency
            self.lastAcceptedPose = acceptedPose
            self._clearPendingPose()
            self._publishPoseFilterStatus("accepted-persistent-jump", jumpMeters, jumpDegrees)
            return (acceptedPose, acceptedLatency)

        self._publishPoseFilterStatus("rejected-spike", jumpMeters, jumpDegrees)
        return (None, None)


    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
        heartbeating = now < self.lastHeartbeatTime + 2  # no heartbeat for 2s => stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating
        if not self.heartbeating:
            self._clearPendingPose()


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name

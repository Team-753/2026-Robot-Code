from wpilib import Timer
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
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

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
        

    def getPoseData(self,yaw) -> tuple[geometry.Pose2d | None, float | None]:
        # robot_orientation_set requires numeric values; coerce Rotation2d or raw numeric yaw to float degrees
        if isinstance(yaw, geometry.Rotation2d):
            self.yaw = float(yaw.degrees())
        else:
            try:
                self.yaw = float(yaw)
            except (TypeError, ValueError):
                self.yaw = 0.0
        """ Returns the *last* calculated robot Pose2D and the pipeline latency, or (None, None) if unavailable """
        self.table.getEntry("robot_orientation_set").setDoubleArray([self.yaw,0.0,0.0,0.0,0.0,0.0])
        bot_pose_data = self.table.getEntry("botpose_orb_wpiblue").getDoubleArray([0.5,0,0,0,0,0])
        if len(bot_pose_data) < 7:
            return (None, None)
        pose_2d = geometry.Pose2d(
            geometry.Translation2d(bot_pose_data[0], bot_pose_data[1]),
            geometry.Rotation2d.fromDegrees(bot_pose_data[5]),
        )
        latency = bot_pose_data[6]
        return (pose_2d, latency)


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


def _fix_name(name: str):
    if not name:
        name = "limelight"
    return name

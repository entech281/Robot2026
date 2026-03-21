package frc.robot.sensors.vision;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionPose {
    private final Pose3d pose;
    private final double timeStamp;
    private final double ambiguity;
    private final String cameraUsed;
    private final double avgDistanceToTags;
    private final List<PhotonTrackedTarget> trackedTargets;

    public VisionPose(Pose3d pose, double timeStamp, double ambiguity, String cameraUsed,
            List<PhotonTrackedTarget> trackedTargets, double avgDistanceToTags) {
        this.timeStamp = timeStamp;
        this.pose = pose;
        this.ambiguity = ambiguity;
        this.cameraUsed = cameraUsed;
        this.trackedTargets = trackedTargets;
        this.avgDistanceToTags = avgDistanceToTags;
    }

    public Pose2d getPose2d() {
        return pose.toPose2d();
    }

    public Pose3d getPose3d() {
        return pose;
    }

    public double getTimeStamp() {
        return timeStamp;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    public String getCameraUsed() {
        return cameraUsed;
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return trackedTargets;
    }

    public double getAvgDistanceToTags() {
        return avgDistanceToTags;
    }
}

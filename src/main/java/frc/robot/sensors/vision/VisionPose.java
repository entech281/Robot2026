package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPose {
    private final Pose2d pose;
    private final double timeStamp;
    private final double ambiguity;
    private final String cameraUsed;

    public VisionPose(Pose2d pose, double timeStamp, double ambiguity, String cameraUsed) {
        this.timeStamp = timeStamp;
        this.pose = pose;
        this.ambiguity = ambiguity;
        this.cameraUsed = cameraUsed;
    }

    public Pose2d getPose() {
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
}

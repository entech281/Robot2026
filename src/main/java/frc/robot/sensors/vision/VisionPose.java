package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPose {
    private final Pose2d pose;
    private final double timeStamp;

    public VisionPose(Pose2d pose, double timeStamp) {
        this.timeStamp = timeStamp;
        this.pose = pose;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimeStamp() {
        return timeStamp;
    }
}

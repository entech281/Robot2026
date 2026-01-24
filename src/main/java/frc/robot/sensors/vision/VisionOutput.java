package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.entech.subsystems.SubsystemOutput;

public class VisionOutput extends SubsystemOutput {
    
    // Basic target information
    public boolean hasTargets = false;
    public int targetId = -1;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    
    // Pose estimation information
    public boolean hasPoseEstimate = false;
    public Pose2d estimatedPose = new Pose2d();
    public double latencyMs = 0.0;
    public int visibleTagCount = 0;
    
    // Camera connection status
    public boolean cameraConnected = false;
    
    @Override
    public void toLog() {
        // Logging can be implemented here if needed
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Vision[");
        
        if (!cameraConnected) {
            sb.append("CAMERA_DISCONNECTED");
        } else if (hasPoseEstimate) {
            sb.append(String.format(
                "POSE: x=%.2fm, y=%.2fm, θ=%.1f°, tags=%d, latency=%.1fms",
                estimatedPose.getX(),
                estimatedPose.getY(),
                estimatedPose.getRotation().getDegrees(),
                visibleTagCount,
                latencyMs
            ));
            
            if (hasTargets) {
                sb.append(String.format(
                    " | TARGET: id=%d, yaw=%.1f°, pitch=%.1f°",
                    targetId, yaw, pitch
                ));
            }
        } else if (hasTargets) {
            sb.append(String.format(
                "TARGET_ONLY: id=%d, yaw=%.1f°, pitch=%.1f°, area=%.2f (NO_POSE)",
                targetId, yaw, pitch, area
            ));
        } else {
            sb.append("NO_TARGETS");
        }
        
        sb.append("]");
        return sb.toString();
    }
    
    public void setTarget(org.photonvision.targeting.PhotonTrackedTarget target) {
        this.hasTargets = true;
        this.targetId = target.getFiducialId();
        this.yaw = target.getYaw();
        this.pitch = target.getPitch();
        this.area = target.getArea();
    }
    
    public void setPoseEstimate(Pose2d pose, int tagCount, double latency) {
        this.hasPoseEstimate = true;
        this.estimatedPose = pose;
        this.visibleTagCount = tagCount;
        this.latencyMs = latency;
    }
}
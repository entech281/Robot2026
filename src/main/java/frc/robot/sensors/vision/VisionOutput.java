package frc.robot.sensors.vision;

import frc.entech.subsystems.SubsystemOutput;

public class VisionOutput extends SubsystemOutput {
    
    public boolean hasTargets = false;
    public int targetId = -1;
    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;
    public double width = 0.0;
    public double height = 0.0;

    @Override
    public void toLog() {
        // Logging can be implemented here if needed
    }

    @Override
    public String toString() {
        return String.format("Vision[hasTargets=%b, height=%2f, width=%2f, yaw=%.2f, pitch=%.2f, area=%.2f, id=%d]",
            hasTargets, height, width, yaw, pitch, area, targetId);
    }

    public void setTarget(org.photonvision.targeting.PhotonTrackedTarget target) {
        this.hasTargets = true;
        this.targetId = target.getFiducialId();
        this.yaw = target.getYaw();
        this.pitch = target.getPitch();
        this.area = target.getArea();
        
    }
}
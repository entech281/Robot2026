package frc.robot.sensors.vision;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.sensors.EntechSensor;

/**
 * PhotonVision subsystem for AprilTag-based pose estimation
 */
public class PhotonVisionSensor extends EntechSensor<VisionOutput> {
    
    public static final String CAMERA_NAME = "OV5647";
    
    private SoloCameraContainer cameraContainer;
    private AprilTagFieldLayout fieldLayout;
    
    @Override
    public void initialize() {
        // Load the AprilTag field layout
        try {
            // loadField doesn't throw IOException in newer versions, just load directly
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
            System.out.println("AprilTag field layout loaded successfully");
        } catch (Exception e) {
            System.err.println("CRITICAL: Failed to load AprilTag field layout: " + e.getMessage());
            e.printStackTrace();
            // Create a minimal fallback layout if loading fails
            fieldLayout = new AprilTagFieldLayout(new java.util.ArrayList<>(), 16.54, 8.21);
        }
        
        // Define where your camera is mounted on the robot/rig
        // *** YOU MUST MEASURE AND UPDATE THESE VALUES ***
        // Example values shown below - camera 0.3m forward, centered, 0.4m high, tilted down 25 degrees
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                0.3,   // X: meters forward from robot center (positive = forward)
                0.0,   // Y: meters left from robot center (positive = left)
                0.4    // Z: meters up from ground (camera height)
            ),
            new Rotation3d(
                0,                      // Roll (rotation around X axis)
                Math.toRadians(-25),    // Pitch (rotation around Y axis, negative = tilted down)
                0                       // Yaw (rotation around Z axis)
            )
        );
        
        // Create the camera container with pose estimation
        cameraContainer = new SoloCameraContainer(CAMERA_NAME, robotToCamera, fieldLayout);
        System.out.println("PhotonVision camera container initialized: " + CAMERA_NAME);
    }
    
    @Override
    public boolean isEnabled() {
        return cameraContainer != null && cameraContainer.isConnected();
    }
    
    @Override
    public VisionOutput toOutputs() {
        VisionOutput output = new VisionOutput();
        
        if (cameraContainer == null) {
            System.err.println("WARNING: Camera container is null");
            return output;
        }
        
        if (!cameraContainer.isConnected()) {
            output.cameraConnected = false;
            return output;
        }
        
        output.cameraConnected = true;
        
        // Get filtered camera results
        var result = cameraContainer.getFilteredResult();
        
        // Get basic target info if targets are visible
        if (result.hasTargets()) {
            var bestTarget = result.getBestTarget();
            output.setTarget(bestTarget);
        }
        
        // Get pose estimate from AprilTags
        Optional<Pose2d> estimatedPose = cameraContainer.getEstimatedPose();
        if (estimatedPose.isPresent()) {
            output.setPoseEstimate(
                estimatedPose.get(),
                cameraContainer.getTargetCount(),
                cameraContainer.getLatency()
            );
        }
        
        return output;
    }
    
    @Override
    public Command getTestCommand() {
        return run(() -> {
            VisionOutput output = toOutputs();
            System.out.println("=== VISION TEST ===");
            System.out.println(output.toString());
            System.out.println("==================");
        }).withName("VisionTest");
    }
    
    @Override
    public void simulationPeriodic() {
        periodic();
    }
    
    @Override
    public void periodic() {
        // Output is calculated in toOutputs()
        // Add any dashboard updates or logging here if needed
    }
}
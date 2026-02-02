package frc.robot.sensors.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

/**
 * PhotonVision subsystem for AprilTag-based pose estimation
 */
public class VisionSensor extends EntechSensor<VisionOutput> {
    private CameraContainerI cameraContainer;
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
        // Example values shown below - camera 0.3m forward, centered, 0.4m high, tilted
        // down 25 degrees
        Transform3d robotToCamera = new Transform3d(
                new Translation3d(
                        0.3, // X: meters forward from robot center (positive = forward)
                        0.0, // Y: meters left from robot center (positive = left)
                        0.4 // Z: meters up from ground (camera height)
                ),
                new Rotation3d(
                        0, // Roll (rotation around X axis)
                        Math.toRadians(-25), // Pitch (rotation around Y axis, negative = tilted down)
                        0 // Yaw (rotation around Z axis)
                ));

        // Create the camera container with pose estimation
        CameraContainerI camA = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_A, robotToCamera,
                fieldLayout);
        System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_A);
        CameraContainerI camB = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_B, robotToCamera,
                fieldLayout);
        System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_B);
        CameraContainerI camC = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_C, robotToCamera,
                fieldLayout);
        System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_C);
        CameraContainerI camD = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_D, robotToCamera,
                fieldLayout);
        System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_D);

        cameraContainer = new MultiCameraContainer(camA, camB, camC, camD);
    }

    @Override
    public boolean isEnabled() {
        return cameraContainer != null && cameraContainer.isConnected();
    }

    @Override
    protected VisionOutput toOutputs() {
        VisionOutput output = new VisionOutput();

        output.setIsDriverMode(cameraContainer.isDriverMode());
        output.setIsConnected(cameraContainer.isConnected());
        output.setHasTargets(cameraContainer.hasTargets());
        output.setUnreadResults(cameraContainer.getAllUnreadResults());

        return output;
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public String getName() {
        return "VisionSensor";
    }
}
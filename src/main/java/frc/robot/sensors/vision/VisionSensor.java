package frc.robot.sensors.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

/**
 * PhotonVision subsystem for AprilTag-based pose estimation
 */
public class VisionSensor extends EntechSensor<VisionOutput> {
    private final static boolean ENABLED = true;
    private CameraContainerI cameraContainerA;
    private CameraContainerI cameraContainerB;
    private CameraContainerI cameraContainerC;
    private CameraContainerI cameraContainerD;
    private CameraContainerI cameraNet;
    private AprilTagFieldLayout fieldLayout;

    @Override
    public void initialize() {

        if (ENABLED) {
            // Load the AprilTag field layout
            try {
                // loadField doesn't throw IOException in newer versions, just load directly
                fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
                System.out.println("AprilTag field layout loaded successfully");
            } catch (Exception e) {
                System.err.println("CRITICAL: Failed to load AprilTag field layout: " + e.getMessage());
                e.printStackTrace();
                // Create a minimal fallback layout if loading fails
                fieldLayout = new AprilTagFieldLayout(new java.util.ArrayList<>(), 16.54, 8.21);
            }

            // Create the camera container with pose estimation
            cameraContainerA = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_A,
                    RobotConstants.Vision.Transforms.robotToCameraA,
                    fieldLayout);
            System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_A);
            cameraContainerB = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_B,
                    RobotConstants.Vision.Transforms.robotToCameraB,
                    fieldLayout);
            System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_B);
            cameraContainerC = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_C,
                    RobotConstants.Vision.Transforms.robotToCameraC,
                    fieldLayout);
            System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_C);
            cameraContainerD = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_D,
                    RobotConstants.Vision.Transforms.robotToCameraD,
                    fieldLayout);
            System.out.println("PhotonVision camera container initialized: " + RobotConstants.Vision.Cameras.CAMERA_D);

            cameraNet = new MultiCameraContainer(cameraContainerA, cameraContainerB, cameraContainerC,
                    cameraContainerD);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    protected VisionOutput toOutputs() {
        VisionOutput output = new VisionOutput();

        if (ENABLED) {
            output.setUnreadResultsA(cameraContainerA.getAllUnreadResults());
            output.setUnreadResultsB(cameraContainerB.getAllUnreadResults());
            output.setUnreadResultsC(cameraContainerC.getAllUnreadResults());
            output.setUnreadResultsD(cameraContainerD.getAllUnreadResults());

            output.setConnected(cameraNet.isConnected());

            Optional<List<VisionPose>> poses = cameraNet.getEstimatedPoses();
            if (poses.isPresent()) {
                output.setVisionPoses(poses.get());
            } else {
                output.setVisionPoses(new java.util.ArrayList<>());
            }
        }

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
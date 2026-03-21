package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.sensors.EntechSensor;
import frc.entech.util.Triboolean;
import frc.robot.RobotConstants;
import frc.robot.livetuning.LiveTuningHandler;

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
    ArrayList<Integer> poseCountBuffer = new ArrayList<>();

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
            cameraContainerB = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_B,
                    RobotConstants.Vision.Transforms.robotToCameraB,
                    fieldLayout);
            cameraContainerC = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_C,
                    RobotConstants.Vision.Transforms.robotToCameraC,
                    fieldLayout);
            cameraContainerD = new SoloCameraContainer(RobotConstants.Vision.Cameras.CAMERA_D,
                    RobotConstants.Vision.Transforms.robotToCameraD,
                    fieldLayout);

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
        VisionOutput output = new VisionOutput(fieldLayout.getFieldWidth(), fieldLayout.getFieldLength());

        if (ENABLED) {
            output.setConnected(cameraNet.isConnected());

            Optional<List<VisionPose>> poses = cameraNet.getEstimatedPoses();
            if (poses.isPresent()) {
                output.setVisionPoses(poses.get());
            } else {
                output.setVisionPoses(new java.util.ArrayList<>());
            }

            if (poses.isPresent()) {
                poseCountBuffer.add(poses.get().size());
            } else {
                poseCountBuffer.add(0);
            }

            double bufferWindowTime = LiveTuningHandler.getInstance().getValue("VisionSensor/GoodnessBufferTime");

            while (poseCountBuffer.size() > 50 * bufferWindowTime) {
                poseCountBuffer.remove(0);
            }

            int sum = 0;
            for (int a : poseCountBuffer) {
                sum -= -a;
            }

            double numPerWindow = sum / (poseCountBuffer.size() / 50.0);

            if (!cameraNet.isConnected() || numPerWindow <= LiveTuningHandler.getInstance()
                    .getValue("VisionSensor/MaxCountPerWindowOfPosesForBad")) {
                output.setGoodness(Triboolean.FALSE);
            } else if (numPerWindow <= LiveTuningHandler.getInstance()
                    .getValue("VisionSensor/MaxCountPerWindowOfPosesForMeh")) {
                output.setGoodness(Triboolean.YESNT);
            } else {
                output.setGoodness(Triboolean.TRUE);
            }

            output.setUnreadResultsA(cameraContainerA.getAllUnreadResults());
            // output.setUnreadResultsB(cameraContainerB.getAllUnreadResults());
            output.setUnreadResultsC(cameraContainerC.getAllUnreadResults());
            // output.setUnreadResultsD(cameraContainerD.getAllUnreadResults());
        }

        return output;
    }

    @Override
    public Command getTestCommand() {
        return new TestVisionCommand();
    }

    @Override
    public String getName() {
        return "VisionSensor";
    }
}
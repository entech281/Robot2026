package frc.robot.subsystems.util;

import java.io.IOException;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PoseCreator {

    private static Pose3d getTagPos(int tagId) {
        
        try {
            AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(".\\src\\main\\java\\frc\\robot\\subsystems\\util\\AprilTags.json");
            return tagLayout.getTagPose(tagId).get();
        } catch (IOException e) {
            DriverStation.reportError("Error reading Aprlitag json. Stacktrace: " + e.getStackTrace(), null);
        }

        return null;
    }

    public static Pose3d estimateRobot (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot) {
        return getTagPos(tagId).plus(aprilTagtoCamera.plus(cameraToRobot)); //TODO: might need a .inverse()
    }

    public static Pose3d estimateTurret (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot, Transform3d robotToTurret) {
        return estimateRobot(tagId, aprilTagtoCamera, cameraToRobot).plus(robotToTurret);
    }
}

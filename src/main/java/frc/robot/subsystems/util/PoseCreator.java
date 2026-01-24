package frc.robot.subsystems.util;

import java.io.IOException;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PoseCreator {

    private Pose3d getTagPos(int tagId) throws IOException{
        AprilTagFieldLayout tagLayout = new AprilTagFieldLayout("./AprilTags.json");

        return tagLayout.getTagPose(tagId).get();
    }


    public Pose3d estimateRobot (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot) throws IOException{
        return getTagPos(tagId).plus(aprilTagtoCamera.plus(cameraToRobot).inverse()); //TODO: not sure about this .inverse()
    }

    public Pose3d estimateTurret (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot, Transform3d robotToTurret) throws IOException{
        return this.estimateRobot(tagId, aprilTagtoCamera, cameraToRobot).plus(robotToTurret);
    }
}

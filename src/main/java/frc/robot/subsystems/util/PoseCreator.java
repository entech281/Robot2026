package frc.robot.subsystems.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PoseCreator {

    private static final Map<Integer, Pose3d> tagPoses = new HashMap<>(Map.of(
        0, new Pose3d(0, 0, 0, new Rotation3d())
    ));

    public Pose3d estimateRobot (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot) {
        return tagPoses.get(tagId).plus(aprilTagtoCamera.plus(cameraToRobot).inverse()); //TODO: not sure about this .inverse()
    }

    public Pose3d estimateTurret (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot, Transform3d robotToTurret) {
        return this.estimateRobot(tagId, aprilTagtoCamera, cameraToRobot).plus(robotToTurret);
    }
}

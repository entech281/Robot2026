package frc.robot.subsystems.util;

import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class PoseCreator {

    private TagPos getTagPos(double tagId) {
        
    }


    public Pose3d estimateRobot (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot) {
        return tagPoses.get(tagId).plus(aprilTagtoCamera.plus(cameraToRobot).inverse()); //TODO: not sure about this .inverse()
    }

    public Pose3d estimateTurret (int tagId, Transform3d aprilTagtoCamera, Transform3d cameraToRobot, Transform3d robotToTurret) {
        return this.estimateRobot(tagId, aprilTagtoCamera, cameraToRobot).plus(robotToTurret);
    }

    private class TagPos {
        public Pose3d pose;
        public Rotation3d rotation;

        public TagPos (Pose3d pose, Rotation3d rotation) {
            this.pose = pose;
            this.rotation = rotation;
        }

        public Pose3d getPose() {
            return this.pose;
        }

        public Rotation3d getRotation() {
            return this.rotation;
        }
    }
}

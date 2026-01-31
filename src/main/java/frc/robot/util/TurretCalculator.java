package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

public class TurretCalculator {
    
    public double calculateTargetTurretAngle(Pose2d target, Pose2d robotPose) {
        //TODO: Find out what happens if I request an angle that is outside
        //of the range of motion
        Translation2d targetTranslation = target.getTranslation().minus(robotPose.getTranslation().plus(RobotConstants.TURRET.TURRET_OFFSET));
        return targetTranslation.getAngle().getDegrees() - robotPose.getRotation().getDegrees();
    }

}

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.io.RobotIO;

public class TurretCalculator {
    
    public static double calculateTargetTurretAngle(Pose2d target) {
        //TODO: Find out what happens if I request and angle that is outside
        //of the range of motion
        Pose2d robotPose = RobotIO.getInstance().getOdometryPose();
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        //TODO: Minus or plus I'm not really sure
        return angleToTarget - robotPose.getRotation().getDegrees();
    }

}

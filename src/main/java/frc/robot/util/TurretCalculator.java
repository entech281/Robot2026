package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

public class TurretCalculator {

    private Pose2d target;
    private Pose2d robotPose;

    public TurretCalculator() {
        this.target = new Pose2d();
        this.robotPose = new Pose2d();
    }

    public TurretCalculator(Pose2d target, Pose2d robotPose) {
        this.target = target;
        this.robotPose = robotPose;
    }
    
    public double calculateTargetTurretAngle() {
        Translation2d targetTranslation = target.getTranslation().minus(robotPose.getTranslation().plus(RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation())));
        return targetTranslation.getAngle().getDegrees() - robotPose.getRotation().getDegrees();
    }

    public boolean isValidTurretAngle(double angle, double toleranceDegrees) {
        double calculatedAngle = calculateTargetTurretAngle();
        return angle >= calculatedAngle - toleranceDegrees && angle <= calculatedAngle + toleranceDegrees;
    }

    public void refresh(Pose2d target, Pose2d robotPose) {
        this.target = target;
        this.robotPose = robotPose;
    }

}

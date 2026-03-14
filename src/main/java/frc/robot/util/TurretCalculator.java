package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
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
        // RobotConstants.
        // target = new Pose2d(target.getX(), -target.getY(), target.getRotation());
        Translation2d targetTranslation = target.getTranslation().plus(
                robotPose.getTranslation().plus(RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation())));

        Translation2d turretToRobot = RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation());
        Pose2d turretPose = new Pose2d(robotPose.getX() - turretToRobot.getX(), robotPose.getY() - turretToRobot.getY(),
                robotPose.getRotation());

        double angle = Math.toDegrees(Math.atan2(target.getY() - turretPose.getY(), target.getX() - turretPose.getX()));

        return ((-robotPose.getRotation().getDegrees()) + (-angle) % 180) + 180;
    }

    public boolean isValidTurretAngle(double angle, double toleranceDegrees) {
        double calculatedAngle = calculateTargetTurretAngle();
        return angle >= calculatedAngle - toleranceDegrees && angle <= calculatedAngle + toleranceDegrees;
    }

    public boolean isValidTurretAngle(Angle angle, Angle toleranceDegrees) {
        return isValidTurretAngle(angle.in(Degrees), toleranceDegrees.in(Degrees));
    }

    public void refresh(Pose2d target, Pose2d robotPose) {
        this.target = target;
        this.robotPose = robotPose;
    }

}

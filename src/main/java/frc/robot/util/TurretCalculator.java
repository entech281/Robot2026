package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;

public class TurretCalculator {

    private Pose2d target;
    private Pose2d robotPose;
    private ChassisSpeeds chassisSpeeds;
    private static final double SPEED_MULTIPLIER = 1.0;

    public TurretCalculator() {
        this.target = new Pose2d();
        this.robotPose = new Pose2d();
        this.chassisSpeeds = new ChassisSpeeds();
    }

    public TurretCalculator(Pose2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
        this.target = target;
        this.robotPose = robotPose;
        this.chassisSpeeds = chassisSpeeds;
    }

    public double calculateTargetTurretAngle() {
        Translation2d turretToRobot = RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation());
        Pose2d turretPose = new Pose2d(robotPose.getX() + turretToRobot.getX(),
                robotPose.getY() + turretToRobot.getY(),
                robotPose.getRotation());

        double angleToTarget = Math
                .toDegrees(Math.atan2(target.getY() - turretPose.getY(), target.getX() - turretPose.getX()));
                // .toDegrees(Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));

        double fieldTurretAngle = angleToTarget - robotPose.getRotation().getDegrees();

        double turretAngleToTarget = ((((-fieldTurretAngle) % 360) + 360) % 360);

        ChassisSpeeds fieldAbsolute = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotPose.getRotation());

        turretAngleToTarget = turretAngleToTarget + (Math.cos(Math.toRadians(angleToTarget)) * (fieldAbsolute.vyMetersPerSecond * SPEED_MULTIPLIER)) + (-Math.sin(Math.toRadians(angleToTarget)) * (fieldAbsolute.vxMetersPerSecond * SPEED_MULTIPLIER));

        return turretAngleToTarget;
    }

    public boolean isValidTurretAngle(double angle, double toleranceDegrees) {
        double calculatedAngle = calculateTargetTurretAngle();
        return angle >= calculatedAngle - toleranceDegrees && angle <= calculatedAngle + toleranceDegrees;
    }

    public boolean isValidTurretAngle(Angle angle, Angle toleranceDegrees) {
        return isValidTurretAngle(angle.in(Degrees), toleranceDegrees.in(Degrees));
    }

    public void refresh(Pose2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
        this.target = target;
        this.robotPose = robotPose;
        this.chassisSpeeds = chassisSpeeds;
    }

}

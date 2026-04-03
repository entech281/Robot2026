package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;

public final class AimingCalculator {
    private AimingCalculator() {

    }

    /**
     * Calculates the necessary turret angle, hood angle, and shooter speed to aim
     * at a target.
     * 
     * Pose3d Heights are used for some calculations but will not affect the height
     * aimed for.
     *
     * @param robotPose   The current pose of the robot.
     * @param targetPose  The pose of the target.
     * @param robotSpeeds The current speeds of the robot.
     * @return The necessary aiming data.
     */
    public static AimingOutputData calculateAimingData(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotSpeeds) {

        return null;
    }

    public static Angle calculateTurretAngle(Pose2d robotPose, Pose2d virtualPose) {
        Translation2d turretToRobot = RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation());
        Pose2d turretPose = new Pose2d(robotPose.getX() + turretToRobot.getX(),
                robotPose.getY() + turretToRobot.getY(),
                robotPose.getRotation());

        double angleToTarget = Math
                .toDegrees(Math.atan2(virtualPose.getY() - turretPose.getY(), virtualPose.getX() - turretPose.getX()));

        double fieldTurretAngle = angleToTarget - robotPose.getRotation().getDegrees();

        double turretAngleToTarget = ((((-fieldTurretAngle) % 360) + 360) % 360);
        return Degrees.of(turretAngleToTarget);
    }
}

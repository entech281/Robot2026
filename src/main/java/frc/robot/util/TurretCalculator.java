package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
        // 1. Turret field pose: apply robot-relative offset to robot pose
        Pose2d turretPose = robotPose.transformBy(
                new Transform2d(RobotConstants.TURRET.TURRET_OFFSET, new Rotation2d()));

        // 2. Target position in turret's coordinate frame
        Translation2d targetInTurretFrame = target.relativeTo(turretPose).getTranslation();

        // 3. CW angle from turret forward to target, normalized to [0, 360)
        double turretAngleDeg = MathUtil.inputModulus(-targetInTurretFrame.getAngle().getDegrees(), 0, 360);

        // 4. Velocity lead compensation
        //    Project field velocity perpendicular to the line of sight.
        //    NOTE: adds m/s directly to degrees -- known dimensional mismatch.
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds, robotPose.getRotation());
        Rotation2d fieldAngleToTarget = target.getTranslation()
                .minus(turretPose.getTranslation()).getAngle();
        double perpendicularVelocity =
                fieldAngleToTarget.getCos() * fieldSpeeds.vyMetersPerSecond
              - fieldAngleToTarget.getSin() * fieldSpeeds.vxMetersPerSecond;
        turretAngleDeg += perpendicularVelocity * SPEED_MULTIPLIER;

        return turretAngleDeg;
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

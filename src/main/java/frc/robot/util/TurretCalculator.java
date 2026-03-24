package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

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
        // Find the turret's pose in the field using the robot's pose and the turret's robot-relative offset.
        Transform2d robotToTurret = new Transform2d(RobotConstants.TURRET.TURRET_OFFSET, new Rotation2d());
        Pose2d turretPose = robotPose.transformBy(robotToTurret);

        // Find the target's pose relative to the turret.
        // The resulting translation's angle is the CCW angle to the target from the turret's forward direction.
        Pose2d targetRelativeToTurret = target.relativeTo(turretPose);
        Rotation2d baseTurretAngle = targetRelativeToTurret.getTranslation().getAngle();

        // Calculate tangential velocity for leading the target.
        // Use WPILib ChassisSpeeds conversion to get field-relative speeds from robot-relative speeds.
        ChassisSpeeds fieldAbsolute = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotPose.getRotation());
        
        // Target's field-relative angle is used to decompose velocity into radial and tangential components.
        Translation2d turretToTarget = target.getTranslation().minus(turretPose.getTranslation());
        Rotation2d fieldAngleToTarget = turretToTarget.getAngle();
        
        // Tangential velocity: V_perp = vy * cos(theta) - vx * sin(theta)
        // Positive tangential velocity means the robot is moving left relative to the target line of sight.
        double tangentialVelocity = (fieldAbsolute.vyMetersPerSecond * fieldAngleToTarget.getCos()) 
                                    - (fieldAbsolute.vxMetersPerSecond * fieldAngleToTarget.getSin());

        // Assuming CCW positive angle for the turret motor:
        // If moving left (tangential velocity > 0), we must shoot right (subtract from the CCW angle).
        double velocityCompensationDegrees = -tangentialVelocity * SPEED_MULTIPLIER;
        
        // Final desired CCW Angle = base angle + velocity compensation
        double turretAngleToTarget = baseTurretAngle.getDegrees() + velocityCompensationDegrees;

        // The user confirmed their turret motor expects CW positive.
        // We must negate `turretAngleToTarget` here:
        turretAngleToTarget = -turretAngleToTarget;

        // Wrap to [0, 360) for convenience, matching the original logic structure but correctly signed
        return (((turretAngleToTarget % 360) + 360) % 360);
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

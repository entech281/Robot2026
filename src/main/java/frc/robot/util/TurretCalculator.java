package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConstants;

/**
 * Calculates the turret angle needed to aim at the hub, accounting for
 * robot motion by leading the target using flight-time estimates.
 *
 * Geometry overview:
 *   1. Compute turret position in field coordinates (robot pose + turret offset).
 *   2. Compute a "virtual target" that accounts for robot velocity during ball flight:
 *      virtualTarget = hub - robotVelocity * flightTime
 *      (The ball inherits the robot's velocity at launch, so we aim opposite the motion.)
 *   3. Compute the field-frame angle from turret to virtual target.
 *   4. Convert to turret-relative angle (subtract robot heading, negate for turret convention).
 */
public class TurretCalculator {

    private static final double DEFAULT_HOOD_ANGLE_DEGREES = 30.0;

    private Pose2d target;
    private Pose2d robotPose;
    private ChassisSpeeds chassisSpeeds;

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

    /**
     * Calculate the turret angle with motion compensation using a default hood angle.
     */
    public double calculateTargetTurretAngle() {
        return calculateTargetTurretAngle(DEFAULT_HOOD_ANGLE_DEGREES);
    }

    /**
     * Calculate the turret angle needed to hit the target, leading for robot motion.
     *
     * @param hoodAngleDegrees the current/commanded hood (launch) angle in degrees,
     *                         used to estimate ball flight time more accurately
     * @return turret angle in degrees (turret-relative, 0-360 convention)
     */
    public double calculateTargetTurretAngle(double hoodAngleDegrees) {
        Translation2d turretFieldPosition = getTurretFieldPosition();
        Translation2d virtualTarget = computeVirtualTarget(turretFieldPosition, hoodAngleDegrees);
        double fieldAngleToTarget = getFieldAngleDegrees(turretFieldPosition, virtualTarget);
        return fieldAngleToTurretAngle(fieldAngleToTarget);
    }

    /**
     * Get the turret's position in field coordinates by applying the turret offset
     * (rotated by the robot's heading) to the robot's pose.
     */
    private Translation2d getTurretFieldPosition() {
        Translation2d rotatedOffset = RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation());
        return new Translation2d(
                robotPose.getX() + rotatedOffset.getX(),
                robotPose.getY() + rotatedOffset.getY());
    }

    /**
     * Compute a virtual target that compensates for robot motion during ball flight.
     *
     * The ball inherits the robot's field velocity at launch. To make it arrive at
     * the actual hub, we aim at: hub - velocity * flightTime. This is equivalent to
     * aiming where the hub "would be" if it were moving opposite to the robot.
     */
    private Translation2d computeVirtualTarget(Translation2d turretFieldPosition, double hoodAngleDegrees) {
        double distanceToTarget = turretFieldPosition.getDistance(target.getTranslation());
        double flightTime = FlightTimeEstimator.getFlightTimeSeconds(distanceToTarget, hoodAngleDegrees);

        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                chassisSpeeds, robotPose.getRotation());

        double virtualX = target.getX() - fieldVelocity.vxMetersPerSecond * flightTime;
        double virtualY = target.getY() - fieldVelocity.vyMetersPerSecond * flightTime;

        return new Translation2d(virtualX, virtualY);
    }

    /**
     * Compute the field-frame angle (degrees) from a position to a target.
     */
    private double getFieldAngleDegrees(Translation2d from, Translation2d to) {
        return Math.toDegrees(Math.atan2(to.getY() - from.getY(), to.getX() - from.getX()));
    }

    /**
     * Convert a field-frame angle to a turret-relative angle.
     *
     * Turret angle is measured as a positive rotation from the robot's heading,
     * using the turret's 0-360 convention (negated and wrapped).
     */
    private double fieldAngleToTurretAngle(double fieldAngleDegrees) {
        double relativeAngle = fieldAngleDegrees - robotPose.getRotation().getDegrees();
        return (((-relativeAngle) % 360) + 360) % 360;
    }

    public boolean isValidTurretAngle(double angle, double toleranceDegrees) {
        double calculatedAngle = calculateTargetTurretAngle();
        return angle >= calculatedAngle - toleranceDegrees && angle <= calculatedAngle + toleranceDegrees;
    }

    public boolean isValidTurretAngle(Angle angle, Angle toleranceDegrees) {
        return isValidTurretAngle(angle.in(Degrees), toleranceDegrees.in(Degrees));
    }

    /**
     * Refresh the calculator state with new sensor data.
     */
    public void refresh(Pose2d target, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
        this.target = target;
        this.robotPose = robotPose;
        this.chassisSpeeds = chassisSpeeds;
    }

    /** Visible for logging/debugging: distance from turret to target in meters. */
    public double getDistanceToTargetMeters() {
        return getTurretFieldPosition().getDistance(target.getTranslation());
    }
}

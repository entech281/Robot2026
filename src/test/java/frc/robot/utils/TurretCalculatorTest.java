package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TurretCalculator;

/**
 * Tests for TurretCalculator, including shoot-while-moving virtual target logic.
 */
public class TurretCalculatorTest {

    private static final double ANGLE_TOLERANCE = 2.0; // degrees

    @BeforeEach
    public void setup() {
        // HAL must be initialized for WPILib units
        try {
            edu.wpi.first.hal.HAL.initialize(500, 0);
        } catch (Exception e) {
            // already initialized
        }
    }

    /**
     * Stationary robot: turret should aim directly at the target (no velocity correction).
     */
    @Test
    public void stationaryRobotAimsAtTarget() {
        Pose2d target = new Pose2d(5.0, 0.0, new Rotation2d());
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        TurretCalculator calc = new TurretCalculator(target, robotPose, speeds);
        double angle = calc.calculateTargetTurretAngle(30.0);

        // Target is straight ahead (+X), robot heading is 0 → turret angle should be ~0 (or ~360)
        double normalized = angle % 360;
        assertTrue(normalized < ANGLE_TOLERANCE || normalized > 360 - ANGLE_TOLERANCE,
                "Stationary robot should aim directly at target ahead. Got: " + angle);
    }

    /**
     * Robot moving laterally (perpendicular to target): turret should lead
     * in the opposite direction of motion.
     */
    @Test
    public void lateralMotionLeadsTarget() {
        Pose2d target = new Pose2d(5.0, 0.0, new Rotation2d());
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        // Robot moving in +Y direction (field frame) at 2 m/s
        ChassisSpeeds movingLeft = new ChassisSpeeds(0, 2.0, 0);
        TurretCalculator calcMoving = new TurretCalculator(target, robotPose, movingLeft);
        double angleMoving = calcMoving.calculateTargetTurretAngle(30.0);

        ChassisSpeeds stationary = new ChassisSpeeds(0, 0, 0);
        TurretCalculator calcStationary = new TurretCalculator(target, robotPose, stationary);
        double angleStationary = calcStationary.calculateTargetTurretAngle(30.0);

        // Moving in +Y should lead in -Y direction (virtual target shifts -Y),
        // so turret angle should differ from stationary
        assertTrue(Math.abs(angleMoving - angleStationary) > 0.5,
                "Lateral motion should cause turret to lead. Moving=" + angleMoving + " Stationary=" + angleStationary);
    }

    /**
     * Robot moving directly toward the target: turret angle should be
     * nearly the same as stationary (no lateral lead needed).
     */
    @Test
    public void radialMotionMinimalAngleChange() {
        Pose2d target = new Pose2d(5.0, 0.0, new Rotation2d());
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        // Moving directly toward target (+X at 2 m/s)
        ChassisSpeeds movingToward = new ChassisSpeeds(2.0, 0, 0);
        TurretCalculator calcMoving = new TurretCalculator(target, robotPose, movingToward);
        double angleMoving = calcMoving.calculateTargetTurretAngle(30.0);

        ChassisSpeeds stationary = new ChassisSpeeds(0, 0, 0);
        TurretCalculator calcStationary = new TurretCalculator(target, robotPose, stationary);
        double angleStationary = calcStationary.calculateTargetTurretAngle(30.0);

        // Moving directly toward target should cause minimal angular change
        double diff = Math.abs(angleMoving - angleStationary);
        if (diff > 180) diff = 360 - diff;
        assertTrue(diff < ANGLE_TOLERANCE,
                "Radial motion should cause minimal turret angle change. Diff=" + diff);
    }

    /**
     * Verify the distance-to-target helper method.
     */
    @Test
    public void distanceToTargetIsCorrect() {
        Pose2d target = new Pose2d(3.0, 4.0, new Rotation2d());
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

        TurretCalculator calc = new TurretCalculator(target, robotPose, speeds);
        double distance = calc.getDistanceToTargetMeters();

        // 3-4-5 triangle, minus turret offset (small, so ~5.0)
        assertTrue(distance > 4.5 && distance < 5.5,
                "Distance should be approximately 5m. Got: " + distance);
    }
}

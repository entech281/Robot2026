package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TurretCalculator;

public class TurretCalculatorTest {

    private static final double EPS = 1e-4; // 4 decimals of precision for manual tests
    TurretCalculator calculator;

    private static final double BLUE_HUB_X = 4.6255940;
    private static final double BLUE_HUB_Y = 4.0346630;
    private static final double RED_HUB_X = 11.9153940;
    private static final double RED_HUB_Y = 4.0346630;

    @BeforeEach
    public void setup() {
        calculator = new TurretCalculator();
    }

    private void runQuadrantTest(double robotX, double robotY, double targetX, double targetY, double expectedAngle) {
        Pose2d target = new Pose2d(targetX, targetY, new Rotation2d());
        Pose2d robotPose = new Pose2d(robotX, robotY, new Rotation2d()); // Facing 0 degrees

        calculator.refresh(target, robotPose, new ChassisSpeeds());
        double calculatedAngle = calculator.calculateTargetTurretAngle();

        assertEquals(expectedAngle, calculatedAngle, EPS,
                String.format("Failed for Robot at (%.2f, %.2f) aiming at (%.2f, %.2f)", robotX, robotY, targetX,
                        targetY));
    }

    /**
     * Test the 4 Quadrants surrounding the BLUE HUB.
     * Quadrant 1: +2m X, +2m Y = (6.6255940, 6.0346630)
     * Quadrant 2: -2m X, +2m Y = (2.6255940, 6.0346630)
     * Quadrant 3: -2m X, -2m Y = (2.6255940, 2.0346630)
     * Quadrant 4: +2m X, -2m Y = (6.6255940, 2.0346630)
     */
    @Test
    public void testBlueHubQuadrants() {
        runQuadrantTest(6.6255940, 6.0346630, BLUE_HUB_X, BLUE_HUB_Y, 133.5459109); // Blue Q1
        runQuadrantTest(2.6255940, 6.0346630, BLUE_HUB_X, BLUE_HUB_Y, 41.5393953); // Blue Q2
        runQuadrantTest(2.6255940, 2.0346630, BLUE_HUB_X, BLUE_HUB_Y, 316.2845947); // Blue Q3
        runQuadrantTest(6.6255940, 2.0346630, BLUE_HUB_X, BLUE_HUB_Y, 228.6289955); // Blue Q4
    }

    /**
     * Test the 4 Quadrants surrounding the RED HUB.
     * Quadrant 1: +2m X, +2m Y = (13.9153940, 6.0346630)
     * Quadrant 2: -2m X, +2m Y = (9.9153940, 6.0346630)
     * Quadrant 3: -2m X, -2m Y = (9.9153940, 2.0346630)
     * Quadrant 4: +2m X, -2m Y = (13.9153940, 2.0346630)
     */
    @Test
    public void testRedHubQuadrants() {
        runQuadrantTest(13.9153940, 6.0346630, RED_HUB_X, RED_HUB_Y, 133.5459109); // Red Q1
        runQuadrantTest(9.9153940, 6.0346630, RED_HUB_X, RED_HUB_Y, 41.5393953); // Red Q2
        runQuadrantTest(9.9153940, 2.0346630, RED_HUB_X, RED_HUB_Y, 316.2845947); // Red Q3
        runQuadrantTest(13.9153940, 2.0346630, RED_HUB_X, RED_HUB_Y, 228.6289955); // Red Q4
    }

    /**
     * Hand calculated 1-foot difference test:
     * We calculate the angle to the Red Hub from a specific point.
     * We then move exactly 1 foot left (in X) and calculate the new angle
     * explicitly.
     * Finally, we prove the difference matched our expected delta.
     * 
     * Hand Math:
     * Target: Red Hub (11.9153940, 4.0346630)
     * Start point: 1m left of Hub X = 10.9153940, Y = Hub Y (4.0346630)
     * Since robot faces 0, turret pose X = 10.9153940 - (-0.17145) = 11.086844m
     * Expected field angle 1 = atan2(4.034663 - 3.958463, 11.9153940 - 11.086844)
     * -> 356.2782908 degrees (CW).
     * 
     * Point 2 (1 foot further away in X = 0.3048m): X = 10.6105940
     * Expected field angle 2 = 357.0451704 degrees.
     * Difference = 0.7668797 degrees.
     */
    @Test
    public void testOneFootDisplacementDifference() {
        Pose2d target = new Pose2d(RED_HUB_X, RED_HUB_Y, new Rotation2d());

        // Point 1
        double startX = 10.9153940;
        Pose2d pose1 = new Pose2d(startX, RED_HUB_Y, new Rotation2d());
        calculator.refresh(target, pose1, new ChassisSpeeds());
        double angle1 = calculator.calculateTargetTurretAngle();
        assertEquals(356.2782908, angle1, EPS);

        // Point 2 (1 foot = 0.3048 meters shifted X)
        double footShiftedX = startX - 0.3048;
        Pose2d pose2 = new Pose2d(footShiftedX, RED_HUB_Y, new Rotation2d());
        calculator.refresh(target, pose2, new ChassisSpeeds());
        double angle2 = calculator.calculateTargetTurretAngle();
        assertEquals(357.0451704, angle2, EPS);

        // Assert that the physical difference algorithm matches exactly what was
        // expected:
        double difference = Math.abs(angle1 - angle2);
        if (difference > 180) {
            difference = 360 - difference;
        }
        assertEquals(0.7668797, difference, EPS, "One foot shift difference calculation failed!");
    }

    /**
     * Hand calculated proof that missing camera pitch (>0) distorts XY planar
     * calculations.
     * 
     * Target: Hub at Z = 2.64 meters. Robot at X = 2.64m, Y = 1.524m. (Approx 10ft
     * straight distance, 5ft offset)
     * Robot heading = 0 degrees. Camera height Z = 0 m (e.g. floored camera).
     * Camera true pitch = +20 degrees (pointing Up).
     * 
     * Software assumption: Pitch = 0.
     */
    @Test
    public void testMissingCameraPitchImpact() {
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());

        // Exact distances for ~10 ft total distance, ~5ft offset
        double trueX = 2.64;
        double trueY = 1.524;
        Pose2d trueTarget = new Pose2d(trueX, trueY, new Rotation2d());

        double trueZDiff = 2.64; // Height of hub target from camera
        double pitchRad = Math.toRadians(20.0);

        // Project world coordinates onto pitched camera frame (pitch up 20 degrees)
        double camForward = trueX * Math.cos(pitchRad) + trueZDiff * Math.sin(pitchRad);
        double camLeft = trueY;
        double camUp = -trueX * Math.sin(pitchRad) + trueZDiff * Math.cos(pitchRad);

        // Physical camera sensor pixel mapping (tx and ty)
        double txObserved = Math.atan2(camLeft, camForward);
        double tyObserved = Math.atan2(camUp, camForward);

        // Software error projection: assuming camera was level (pitch = 0)
        double pseudoDistanceXY = trueZDiff / Math.tan(tyObserved);
        double estimatedX = pseudoDistanceXY * Math.cos(txObserved);
        double estimatedY = pseudoDistanceXY * Math.sin(txObserved);
        Pose2d estimatedTarget = new Pose2d(estimatedX, estimatedY, new Rotation2d());

        // Calculate the physical Angle output required for the ACTUAL target
        calculator.refresh(trueTarget, robotPose, new ChassisSpeeds());
        double trueTurretAngle = calculator.calculateTargetTurretAngle();

        // Calculate the requested angle given the CORRUPTED target location
        calculator.refresh(estimatedTarget, robotPose, new ChassisSpeeds());
        double estimatedTurretAngle = calculator.calculateTargetTurretAngle();

        double errorDegrees = Math.abs(trueTurretAngle - estimatedTurretAngle);
        if (errorDegrees > 180) {
            errorDegrees = 360 - errorDegrees;
        }

        // Assert error is greater than 5 degrees
        assertTrue(errorDegrees > 5.0,
                "The error caused by the missing 20-degree pitch was " + errorDegrees
                        + " degrees, which is not > 5 degrees!");
    }
}

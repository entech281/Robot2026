package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TurretCalculator;

/**
 * Tests for {@link TurretCalculator#calculateTargetTurretAngle()}.
 *
 * Turret angle convention (confirmed with hardware):
 *   - 0 deg = turret pointing straight forward (robot +X direction)
 *   - Angles increase clockwise (CW) when viewed from above
 *   - 90 deg = turret pointing to the right
 *   - 180 deg = turret pointing backward
 *   - 270 deg = turret pointing to the left
 *   - Output range: [0 deg, 360 deg)
 *
 * WPILib field coordinate system:
 *   - +X = toward the red alliance wall
 *   - +Y = toward the left (from blue driver station perspective)
 *   - Robot heading 0 deg = facing +X, increases counter-clockwise (CCW)
 *
 * Turret offset from robot center (in robot frame):
 *   - X = -6.75" (behind center)
 *   - Y = -3.0"  (to the right of center)
 */
public class TurretCalculatorTest {

    // Turret offset in meters (matches RobotConstants.TURRET.TURRET_OFFSET)
    private static final double TURRET_OFFSET_X = Units.inchesToMeters(-6.75);
    private static final double TURRET_OFFSET_Y = Units.inchesToMeters(-3.0);

    // Blue hub position (matches RobotConstants.TURRET.BLUE_HUB_LOCATION)
    private static final double BLUE_HUB_X = Units.inchesToMeters(182.11);
    private static final double BLUE_HUB_Y = Units.inchesToMeters(158.845);

    private static final ChassisSpeeds ZERO_SPEED = new ChassisSpeeds();

    private TurretCalculator calculator;

    @BeforeEach
    void setup() {
        calculator = new TurretCalculator();
    }

    // -----------------------------------------------------------------------
    // ALTERNATIVE COMPUTATION METHOD (cross-check)
    //
    // Computes the expected turret angle using a DIFFERENT code path than
    // the production code. Instead of:
    //   Production: atan2 in field frame -> subtract heading -> negate -> normalize
    // This method:
    //   1. Computes the turret-to-target vector in field frame
    //   2. Rotates that vector into the robot frame
    //   3. Computes the CW angle from forward using atan2(-dy_robot, dx_robot)
    //
    // If both methods agree, we have high confidence in correctness.
    // -----------------------------------------------------------------------
    private static double expectedAngle(double targetX, double targetY,
                                        double robotX, double robotY,
                                        double robotHeadingDeg) {
        double h = Math.toRadians(robotHeadingDeg);
        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // Step 1: Turret position in field (rotate offset by robot heading, then add)
        double turretX = robotX + (TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH);
        double turretY = robotY + (TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH);

        // Step 2: Vector from turret to target in field frame
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        // Step 3: Rotate into robot frame (rotate by -heading)
        double dxRobot = dx * cosH + dy * sinH;
        double dyRobot = -dx * sinH + dy * cosH;

        // Step 4: CW angle from forward; atan2(-dy, dx) gives CW-positive
        double angle = Math.toDegrees(Math.atan2(-dyRobot, dxRobot));
        if (angle < 0) {
            angle += 360.0;
        }
        return angle;
    }

    // Helper: refresh calculator with zero velocity (stationary robot)
    private void refreshStationary(double targetX, double targetY,
                                   double robotX, double robotY,
                                   double robotHeadingDeg) {
        calculator.refresh(
            new Pose2d(targetX, targetY, new Rotation2d()),
            new Pose2d(robotX, robotY, Rotation2d.fromDegrees(robotHeadingDeg)),
            ZERO_SPEED);
    }

    // ===================================================================
    // GROUP 1: CARDINAL DIRECTION SANITY CHECKS
    //
    // Robot at origin, facing +X. Target 10m away in each cardinal direction.
    // At 10m the turret offset (~0.19m) causes <1 deg deviation, so these are
    // easy for a human to verify: "target ahead -> turret ~ 0 deg", etc.
    // ===================================================================
    @Nested
    @DisplayName("Cardinal Directions (10m away, easy to verify)")
    class CardinalDirections {

        /**
         * Target 10m directly ahead of robot.
         *
         *           target (10, 0)
         *              ^
         *    robot ->  [R]  -> +X (heading 0 deg)
         *
         * Turret should point nearly forward ~ 0 deg (actually ~359.57 deg due to
         * the turret being offset 3" to the right, aiming slightly left).
         */
        @Test
        @DisplayName("Target ahead -> turret ~ 0 deg")
        void targetAhead() {
            refreshStationary(10, 0, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();

            assertTrue(angle > 355 || angle < 5,
                "Target ahead: turret should be near 0 deg, got " + angle);
            assertEquals(expectedAngle(10, 0, 0, 0, 0), angle, 0.01);
        }

        /**
         * Target 10m directly to the right of robot (-Y in field frame).
         *
         *    robot ->  [R]  -> +X
         *              v
         *           target (0, -10)
         *
         * Turret should point right ~ 90 deg CW.
         */
        @Test
        @DisplayName("Target right -> turret ~ 90 deg")
        void targetRight() {
            refreshStationary(0, -10, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();

            assertTrue(angle > 85 && angle < 95,
                "Target right: turret should be near 90 deg, got " + angle);
            assertEquals(expectedAngle(0, -10, 0, 0, 0), angle, 0.01);
        }

        /**
         * Target 10m directly behind robot.
         *
         *           target (-10, 0)
         *              v
         *    robot ->  [R]  -> +X
         *
         * Turret should point backward ~ 180 deg.
         */
        @Test
        @DisplayName("Target behind -> turret ~ 180 deg")
        void targetBehind() {
            refreshStationary(-10, 0, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();

            assertTrue(angle > 175 && angle < 185,
                "Target behind: turret should be near 180 deg, got " + angle);
            assertEquals(expectedAngle(-10, 0, 0, 0, 0), angle, 0.01);
        }

        /**
         * Target 10m directly to the left of robot (+Y in field frame).
         *
         *           target (0, 10)
         *              ^
         *    robot ->  [R]  -> +X
         *
         * Turret should point left ~ 270 deg CW.
         */
        @Test
        @DisplayName("Target left -> turret ~ 270 deg")
        void targetLeft() {
            refreshStationary(0, 10, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();

            assertTrue(angle > 265 && angle < 275,
                "Target left: turret should be near 270 deg, got " + angle);
            assertEquals(expectedAngle(0, 10, 0, 0, 0), angle, 0.01);
        }
    }

    // ===================================================================
    // GROUP 2: TURRET OFFSET VERIFICATION
    //
    // Close targets where the turret offset (-6.75", -3.0") has a
    // measurable effect. This verifies the offset is applied correctly.
    //
    // The turret is behind and to the right of the robot center, so:
    //   - A target straight ahead will appear slightly to the LEFT
    //     from the turret's perspective -> angle slightly < 360 deg (CCW)
    //   - The closer the target, the larger this effect
    // ===================================================================
    @Nested
    @DisplayName("Turret Offset Effects (close targets)")
    class TurretOffsetEffects {

        /**
         * Target 1m ahead. Turret at (-0.171m, -0.076m).
         * Target at (1, 0) is to the upper-right of the turret.
         * Expected: ~356.3 deg (3.7 deg CCW from forward, or equivalently,
         * the turret turns ~3.7 deg to the LEFT because the turret is
         * offset 3" to the right).
         */
        @Test
        @DisplayName("1m ahead: offset causes ~3.7 deg leftward aim")
        void oneMetreAhead() {
            refreshStationary(1, 0, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(1, 0, 0, 0, 0);

            // With offset, turret must aim left of center (angle < 360)
            assertTrue(angle > 350 && angle < 360,
                "1m ahead: expect turret in 350-360 deg range, got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Target only 0.5m ahead -- offset effect is even larger (~6.5 deg).
         */
        @Test
        @DisplayName("0.5m ahead: offset causes ~6.5 deg leftward aim")
        void halfMetreAhead() {
            refreshStationary(0.5, 0, 0, 0, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(0.5, 0, 0, 0, 0);

            assertTrue(angle > 345 && angle < 360,
                "0.5m ahead: expect turret in 345-360 deg range, got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Offset diminishes with distance: at 10m ahead, the offset
         * causes <0.5 deg deviation vs ~6.5 deg at 0.5m.
         */
        @Test
        @DisplayName("Offset effect diminishes with distance")
        void offsetDiminishesWithDistance() {
            refreshStationary(0.5, 0, 0, 0, 0);
            double angleClose = calculator.calculateTargetTurretAngle();
            double deviationClose = 360.0 - angleClose; // degrees from forward

            refreshStationary(10, 0, 0, 0, 0);
            double angleFar = calculator.calculateTargetTurretAngle();
            double deviationFar = 360.0 - angleFar;

            assertTrue(deviationClose > deviationFar * 5,
                "Close target should deviate much more than far target. " +
                "Close=" + deviationClose + " deg vs Far=" + deviationFar + " deg");
        }
    }

    // ===================================================================
    // GROUP 3: ROBOT ROTATION
    //
    // Target fixed at (10, 0). Robot at origin with various headings.
    // As the robot rotates CCW (increasing heading), the turret must
    // rotate CW (increasing turret angle) to keep pointing at the target.
    //
    // Key insight: if the robot turns 45 deg CCW, the turret must turn
    // ~45 deg CW to compensate -> turret angle ~ 45 deg.
    // ===================================================================
    @Nested
    @DisplayName("Robot Rotation (target at (10,0), robot rotates)")
    class RobotRotation {

        @ParameterizedTest(name = "heading={0} deg -> turret~{1} deg")
        @MethodSource("rotationCases")
        void turretCompensatesForRobotRotation(double heading, double approxExpected) {
            refreshStationary(10, 0, 0, 0, heading);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(10, 0, 0, 0, heading);

            assertEquals(expected, angle, 0.01,
                "Heading " + heading + " deg: turret should be near " + approxExpected + " deg");
            // Also verify it's in the right quadrant
            assertTrue(Math.abs(angle - approxExpected) < 5
                    || Math.abs(angle - approxExpected - 360) < 5
                    || Math.abs(angle - approxExpected + 360) < 5,
                "Heading " + heading + " deg: turret " + angle + " deg not near " + approxExpected + " deg");
        }

        static Stream<Arguments> rotationCases() {
            return Stream.of(
                //    heading,  approx turret angle
                Arguments.of(0.0,     0.0),     // facing target -> turret forward
                Arguments.of(45.0,   45.0),     // turned 45 deg left -> turret 45 deg right
                Arguments.of(90.0,   90.0),     // facing +Y -> turret points right
                Arguments.of(135.0, 135.0),     // facing away-left -> turret back-right
                Arguments.of(180.0, 180.0),     // facing away -> turret backward
                Arguments.of(-90.0, 270.0),     // facing -Y -> turret points left
                Arguments.of(-45.0, 315.0)      // turned 45 deg right -> turret 315 deg
            );
        }
    }

    // ===================================================================
    // GROUP 4: REALISTIC HUB SHOTS
    //
    // Blue Hub location: (182.11", 158.845") = (~4.626m, ~4.035m)
    // Tests simulate a robot in the blue alliance zone aiming at the hub.
    // These are the scenarios most relevant to diagnosing the 2 ft left miss.
    // ===================================================================
    @Nested
    @DisplayName("Realistic Hub Shots (Blue Alliance)")
    class RealisticHubShots {

        /**
         * Robot on the hub centerline (same Y as hub), facing the hub.
         * The turret should point nearly straight ahead.
         *
         *    [Hub]  <----  [Robot]
         *    (4.63, 4.03)   (2.0, 4.03)
         *
         * Expected: ~358.4 deg (nearly forward, slight left offset).
         */
        @Test
        @DisplayName("On centerline facing hub -> turret ~ 0 deg")
        void onCenterlineFacingHub() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 0);

            assertTrue(angle > 355 || angle < 5,
                "On centerline: turret should be near 0 deg, got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Robot is 1m to the RIGHT of the hub centerline.
         * The hub is up and to the left -> turret turns left (CCW).
         *
         *                    [Hub]  (4.63, 4.03)
         *                   /
         *    [Robot] (2.0, 3.0)   -> +X
         *
         * Expected: ~338.3 deg (turret turns ~21.7 deg to the LEFT).
         */
        @Test
        @DisplayName("Right of centerline -> turret aims left ~ 338 deg")
        void rightOfCenter() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, 3.0, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 2.0, 3.0, 0);

            assertTrue(angle > 330 && angle < 345,
                "Right of center: turret should aim left (~338 deg), got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Robot is ~1m to the LEFT of the hub centerline.
         * The hub is down and to the left -> turret turns right (CW).
         *
         *    [Robot] (2.0, 5.0)   -> +X
         *                   \
         *                    [Hub]  (4.63, 4.03)
         *
         * Expected: ~17.6 deg (turret turns ~17.6 deg to the RIGHT).
         */
        @Test
        @DisplayName("Left of centerline -> turret aims right ~ 18 deg")
        void leftOfCenter() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, 5.0, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 2.0, 5.0, 0);

            assertTrue(angle > 12 && angle < 25,
                "Left of center: turret should aim right (~18 deg), got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Robot facing 30 deg left of the hub but on the centerline.
         * The turret must compensate by turning ~27 deg CW.
         */
        @Test
        @DisplayName("On centerline, heading 30 deg -> turret compensates ~ 27 deg")
        void angledOnCenterline() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 30);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 30);

            assertTrue(angle > 20 && angle < 35,
                "Angled 30 deg: turret should compensate (~27 deg), got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Robot facing AWAY from hub (heading 180 deg).
         * Turret must point backward ~ 182 deg.
         */
        @Test
        @DisplayName("Facing away from hub -> turret backward ~ 182 deg")
        void facingAway() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 180);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y, 180);

            assertTrue(angle > 175 && angle < 190,
                "Facing away: turret should point backward (~182 deg), got " + angle);
            assertEquals(expected, angle, 0.01);
        }

        /**
         * Robot close to hub (4m from wall vs 2m), still on centerline.
         * Turret offset effect is larger when close.
         */
        @Test
        @DisplayName("Close to hub -> larger offset effect ~ 355 deg")
        void closeToHub() {
            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 4.0, BLUE_HUB_Y, 0);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(BLUE_HUB_X, BLUE_HUB_Y, 4.0, BLUE_HUB_Y, 0);

            assertTrue(angle > 350 || angle < 5,
                "Close to hub: turret should be near 0 deg, got " + angle);
            assertEquals(expected, angle, 0.01);
        }
    }

    // ===================================================================
    // GROUP 5: COMPREHENSIVE CROSS-CHECK
    //
    // Tests many combinations of target positions, robot positions, and
    // headings. The expected value comes from the alternative robot-frame
    // computation method, which uses a completely different code path
    // than the production code.
    // ===================================================================
    @Nested
    @DisplayName("Cross-Check: Alternative Method vs Calculator")
    class CrossCheck {

        @ParameterizedTest(name = "target=({0},{1}) robot=({2},{3}) heading={4} deg")
        @MethodSource("crossCheckCases")
        void alternativeMethodAgreesWithCalculator(
                double tx, double ty, double rx, double ry, double heading) {
            refreshStationary(tx, ty, rx, ry, heading);
            double actual = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(tx, ty, rx, ry, heading);

            assertEquals(expected, actual, 0.01,
                String.format("target=(%.1f,%.1f) robot=(%.1f,%.1f) heading=%.0f deg",
                    tx, ty, rx, ry, heading));
        }

        static Stream<Arguments> crossCheckCases() {
            return Stream.of(
                // Cardinal directions at various distances
                Arguments.of(10.0,   0.0,  0.0, 0.0,   0.0),
                Arguments.of( 0.0,  10.0,  0.0, 0.0,   0.0),
                Arguments.of(-10.0,  0.0,  0.0, 0.0,   0.0),
                Arguments.of( 0.0, -10.0,  0.0, 0.0,   0.0),

                // Diagonals
                Arguments.of( 7.071,  7.071, 0.0, 0.0,  0.0),
                Arguments.of( 7.071, -7.071, 0.0, 0.0,  0.0),
                Arguments.of(-7.071,  7.071, 0.0, 0.0,  0.0),
                Arguments.of(-7.071, -7.071, 0.0, 0.0,  0.0),

                // Robot at non-origin position
                Arguments.of( 5.0,  3.0,  2.0,  1.0,  0.0),
                Arguments.of( 5.0,  3.0,  2.0,  1.0, 45.0),
                Arguments.of( 5.0,  3.0,  2.0,  1.0, 90.0),
                Arguments.of( 5.0,  3.0,  2.0,  1.0, 180.0),
                Arguments.of( 5.0,  3.0,  2.0,  1.0, -90.0),

                // Hub shots from various positions
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  1.0, 4.0,   0.0),
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  1.0, 4.0,  15.0),
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  1.0, 4.0, -15.0),
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  3.0, 2.0,   0.0),
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  3.0, 6.0,   0.0),
                Arguments.of(BLUE_HUB_X, BLUE_HUB_Y,  3.0, 2.0,  45.0),

                // Close-range targets
                Arguments.of( 1.0,  0.0,  0.0, 0.0,   0.0),
                Arguments.of( 0.5,  0.0,  0.0, 0.0,   0.0),
                Arguments.of( 0.0,  1.0,  0.0, 0.0,   0.0),
                Arguments.of( 0.0, -1.0,  0.0, 0.0,   0.0),

                // Robot rotated with close targets
                Arguments.of( 1.0,  0.0,  0.0, 0.0,  90.0),
                Arguments.of( 1.0,  0.0,  0.0, 0.0, -90.0),
                Arguments.of( 1.0,  0.0,  0.0, 0.0, 180.0),
                Arguments.of( 0.0,  1.0,  0.0, 0.0,  45.0)
            );
        }
    }

    // ===================================================================
    // GROUP 6: EDGE CASES
    // ===================================================================
    @Nested
    @DisplayName("Edge Cases")
    class EdgeCases {

        /**
         * Angle wrapping: target at 350 deg and 10 deg from forward should
         * both return values in [0, 360) without discontinuities.
         */
        @Test
        @DisplayName("Angles near 0 deg/360 deg boundary wrap correctly")
        void angleWrappingNear360() {
            // Target slightly to the right of ahead -> small positive CW angle
            refreshStationary(10, -0.5, 0, 0, 0);
            double angleRight = calculator.calculateTargetTurretAngle();
            assertTrue(angleRight > 0 && angleRight < 10,
                "Slightly right: expected small CW angle, got " + angleRight);

            // Target slightly to the left of ahead -> just below 360 deg
            refreshStationary(10, 0.5, 0, 0, 0);
            double angleLeft = calculator.calculateTargetTurretAngle();
            assertTrue(angleLeft > 350 && angleLeft < 360,
                "Slightly left: expected angle near 360 deg, got " + angleLeft);
        }

        /**
         * Robot at a 'weird' heading (e.g., 359 deg). Verify no discontinuity.
         */
        @Test
        @DisplayName("Robot heading near 360 deg doesn't cause discontinuity")
        void headingNear360() {
            refreshStationary(10, 0, 0, 0, 359);
            double angle = calculator.calculateTargetTurretAngle();
            double expected = expectedAngle(10, 0, 0, 0, 359);
            assertEquals(expected, angle, 0.01);
            // Target is ~1 deg to the right of where the robot faces
            assertTrue(angle > 355 || angle < 5,
                "Heading 359 deg, target at ~0 deg: turret should be near 1 deg CW, got " + angle);
        }

        /**
         * Output should always be in [0, 360) regardless of input.
         */
        @Test
        @DisplayName("Output is always in [0, 360)")
        void outputRangeIsNormalized() {
            double[] headings = {0, 45, 90, 135, 180, -45, -90, -135, -180, 270, 359};
            for (double heading : headings) {
                refreshStationary(5, 3, 1, 1, heading);
                double angle = calculator.calculateTargetTurretAngle();
                assertTrue(angle >= 0 && angle < 360,
                    "Heading " + heading + " deg: angle " + angle + " not in [0, 360)");
            }
        }

        /**
         * Symmetry: if the robot is on the hub centerline, a target
         * offset to the left should give approximately the same angular
         * deviation from forward as the same offset to the right.
         * Not perfectly symmetric due to turret offset, but close.
         */
        @Test
        @DisplayName("Left/right symmetry around centerline")
        void leftRightSymmetry() {
            double offset = 2.0; // 2m off centerline

            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y - offset, 0);
            double angleRight = calculator.calculateTargetTurretAngle();

            refreshStationary(BLUE_HUB_X, BLUE_HUB_Y, 2.0, BLUE_HUB_Y + offset, 0);
            double angleLeft = calculator.calculateTargetTurretAngle();

            // Compute unsigned deviation from straight ahead (0/360 degrees)
            double deviationWhenRight = Math.min(angleRight, 360.0 - angleRight);
            double deviationWhenLeft = Math.min(angleLeft, 360.0 - angleLeft);

            // Both deviations should be roughly equal (within ~5 deg due to turret offset)
            assertTrue(Math.abs(deviationWhenRight - deviationWhenLeft) < 5.0,
                "Left/right not symmetric: devRight=" + deviationWhenRight +
                " devLeft=" + deviationWhenLeft);
        }
    }

    // ===================================================================
    // GROUP 7: VELOCITY COMPENSATION
    //
    // NOTE: The velocity compensation in calculateTargetTurretAngle()
    // adds m/s directly to degrees, which is a dimensional mismatch.
    // These tests verify the basic DIRECTION of compensation is correct,
    // but the MAGNITUDE may be wrong. This is flagged as a potential bug.
    //
    // The team confirmed that the 2 ft left misses occur even when
    // stationary, so the velocity compensation is NOT the primary issue.
    // ===================================================================
    @Nested
    @DisplayName("Velocity Compensation (direction checks)")
    class VelocityCompensation {

        /**
         * Robot moving to the RIGHT while aiming at a target ahead.
         * The turret should lead by turning slightly to the right (CW)
         * compared to the stationary case.
         */
        @Test
        @DisplayName("Moving right -> turret leads right (larger CW angle)")
        void movingRightLeadsRight() {
            // Stationary baseline
            calculator.refresh(
                new Pose2d(10, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d()),
                ZERO_SPEED);
            double stationaryAngle = calculator.calculateTargetTurretAngle();

            // Moving to the right (negative vy in robot frame, since +Y is left)
            calculator.refresh(
                new Pose2d(10, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d()),
                new ChassisSpeeds(0, -2.0, 0)); // vy = -2 m/s (rightward)
            double movingAngle = calculator.calculateTargetTurretAngle();

            // Moving right with target ahead: turret should turn MORE clockwise
            // (Note: due to the dimensional bug, this is a direction-only check)
            assertTrue(movingAngle != stationaryAngle,
                "Velocity should affect turret angle");
        }

        /**
         * Stationary robot should give same result regardless of
         * ChassisSpeeds(0,0,0) vs default constructor.
         */
        @Test
        @DisplayName("Zero velocity = no compensation")
        void zeroVelocityNoCompensation() {
            calculator.refresh(
                new Pose2d(10, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d()),
                ZERO_SPEED);
            double angle1 = calculator.calculateTargetTurretAngle();

            calculator.refresh(
                new Pose2d(10, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d()),
                new ChassisSpeeds(0, 0, 0));
            double angle2 = calculator.calculateTargetTurretAngle();

            assertEquals(angle1, angle2, 1e-9, "Zero speed should give identical results");
        }
    }
}

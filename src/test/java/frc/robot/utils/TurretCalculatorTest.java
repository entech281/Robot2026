// package frc.robot.utils;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Radians;
// import static org.junit.jupiter.api.Assertions.assertEquals;

// import java.util.stream.Stream;

// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.params.ParameterizedTest;
// import org.junit.jupiter.params.provider.Arguments;
// import org.junit.jupiter.params.provider.MethodSource;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.RobotConstants;
// import frc.robot.commands.FaceTargetLocationTurretCommand;
// import frc.robot.util.TurretCalculator;

// /**
// * Basic unit tests for {@link FaceTargetLocationTurretCommand}.
// *
// * These tests illustrate the testing framework and how to mock/replace
// * the turret subsystem and the odometry pose provided by RobotIO.
// */
// public class TurretCalculatorTest {

// // tiny tolerance for floating point comparisons
// private static final double EPS = 1e-6;
// TurretCalculator calculator;

// @BeforeEach
// public void setup() {
// // reset odometry pose to a known value before each test
// calculator = new TurretCalculator();
// }

// @ParameterizedTest
// @MethodSource("testCalculateTargetAngleSupplier")
// public void testCalculateTargetAngle(Pose2d target, Pose2d robotPose, double
// expectedAngle) {
// calculator.refresh(target, robotPose);
// double calculatedAngle = calculator.calculateTargetTurretAngle();
// assertEquals(expectedAngle, calculatedAngle, EPS,
// "Calculated turret angle should match expected angle.");

// }

// public static Stream<Arguments> testCalculateTargetAngleSupplier() {
// return Stream.of(

// Arguments.of(new Pose2d(0.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()), 0.0),
// Arguments.of(new Pose2d(1.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()), 0.0),
// Arguments.of(new Pose2d(-1.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()), 180.0),
// //Ones above this I'm more sure of than the ones below because
// //the ones below have math in them from ChatGPT and I had to fix some
// //The above ones will pass so long as the turret offset is zero
// //in the y direction (ie centered on the robot)

// Arguments.of(new Pose2d(0.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()),
// Math.toDegrees(Math.atan2(-RobotConstants.TURRET.TURRET_OFFSET.getY(),
// -RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(1.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()),
// Math.toDegrees(Math.atan2(-RobotConstants.TURRET.TURRET_OFFSET.getY(), 1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(0.0, 1.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()), Math.toDegrees(Math.atan2(1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(),
// -RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// // \/\/This is negative because +/-180 deg are the same
// Arguments.of(new Pose2d(-1.0, 0.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()),
// -Math.toDegrees(Math.atan2(-RobotConstants.TURRET.TURRET_OFFSET.getY(), -1.0
// - RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(0.0, -1.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()), Math.toDegrees(Math.atan2(-1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(),
// -RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(1.0, 1.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d()), Math.toDegrees(Math.atan2(1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(), 1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(-1.0, 1.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()), Math.toDegrees(Math.atan2(1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(), -1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(-1.0, -1.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()), Math.toDegrees(Math.atan2(-1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(), -1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getX()))),
// Arguments.of(new Pose2d(1.0, -1.0, new Rotation2d()), new Pose2d(0.0, 0.0,
// new Rotation2d()), Math.toDegrees(Math.atan2(-1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getY(), 1.0 -
// RobotConstants.TURRET.TURRET_OFFSET.getX()))),

// // Robot rotated cases
// Arguments.of(new Pose2d(0.0, 1.0, new Rotation2d()), new Pose2d(0.0, 0.0, new
// Rotation2d(Degrees.of(90.0).in(Radians))), Math.toDegrees(Math.atan2(1.0 +
// RobotConstants.TURRET.TURRET_OFFSET.rotateBy(new
// Rotation2d(Degrees.of(90.0).in(Radians))).getY(),
// RobotConstants.TURRET.TURRET_OFFSET.rotateBy(new
// Rotation2d(Degrees.of(90.0).in(Radians))).getX())) + 90.0)
// );
// }

// }

package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.commands.FaceTargetLocationTurretCommand;
import frc.robot.util.TurretCalculator;

/**
 * Basic unit tests for {@link FaceTargetLocationTurretCommand}.
 *
 * These tests illustrate the testing framework and how to mock/replace
 * the turret subsystem and the odometry pose provided by RobotIO.
 */
public class TurretCalculatorTest {

  // tiny tolerance for floating point comparisons
  private static final double EPS = 1e-6;
  TurretCalculator calculator;

  @BeforeEach
  public void setup() {
    // reset odometry pose to a known value before each test
    calculator = new TurretCalculator();
  }

  @ParameterizedTest
  @MethodSource("testCalculateTargetAngleSupplier")
  public void testCalculateTargetAngle(Pose2d target, double expectedAngle, Pose2d robotPose) {
    double calculatedAngle = calculator.calculateTargetTurretAngle(target, robotPose);
    assertEquals(expectedAngle, calculatedAngle, EPS,
        "Calculated turret angle should match expected angle.");

  }

public static Stream<Arguments> testCalculateTargetAngleSupplier() {
  return Stream.of(
    Arguments.of(new Pose2d(0.0, 0.0, new Rotation2d()), 0.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(1.0, 0.0, new Rotation2d()), 0.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(0.0, 1.0, new Rotation2d()), 90.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(-1.0, 0.0, new Rotation2d()), 180.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(0.0, -1.0, new Rotation2d()), -90.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(1.0, 1.0, new Rotation2d()), 45.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(-1.0, 1.0, new Rotation2d()), 135.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(-1.0, -1.0, new Rotation2d()), -135.0, new Pose2d(0.0, 0.0, new Rotation2d())),
    Arguments.of(new Pose2d(1.0, -1.0, new Rotation2d()), -45.0, new Pose2d(0.0, 0.0, new Rotation2d())),

    // Robot rotated cases (robot heading affects turret angle)
    Arguments.of(new Pose2d(1.0, 0.0, new Rotation2d()), -90.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(90.0).in(Radians)))),
    Arguments.of(new Pose2d(0.0, 1.0, new Rotation2d()), 0.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(90.0).in(Radians)))),
    Arguments.of(new Pose2d(-1.0, 0.0, new Rotation2d()), 90.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(90.0).in(Radians)))),
    Arguments.of(new Pose2d(1.0, 0.0, new Rotation2d()), -180.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(180.0).in(Radians)))),
    Arguments.of(new Pose2d(0.0, -1.0, new Rotation2d()), -180.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(90.0).in(Radians)))),
    Arguments.of(new Pose2d(1.0, 1.0, new Rotation2d()), 90.0, new Pose2d(0.0, 0.0, new Rotation2d(Degrees.of(-45.0).in(Radians)))),

    // Actual hub positions
    Arguments.of(RobotConstants.TURRET.BLUE_HUB_LOCATION, 0.0, new Pose2d(0.0, Inches.of(158.845).in(Meters), new Rotation2d())),
    Arguments.of(RobotConstants.TURRET.BLUE_HUB_LOCATION, 90.0, new Pose2d(Inches.of(182.11).in(Meters), 0.0, new Rotation2d())),
    Arguments.of(RobotConstants.TURRET.BLUE_HUB_LOCATION, -180.0, new Pose2d(0.0, Inches.of(158.845).in(Meters), new Rotation2d(Degrees.of(180.0).in(Radians)))),
    Arguments.of(RobotConstants.TURRET.BLUE_HUB_LOCATION, -90.0, new Pose2d(Inches.of(182.11).in(Meters), Inches.of(158.845).in(Meters) * 2, new Rotation2d())),

    Arguments.of(RobotConstants.TURRET.RED_HUB_LOCATION, 0.0, new Pose2d(0.0, Inches.of(158.845).in(Meters), new Rotation2d())),
    Arguments.of(RobotConstants.TURRET.RED_HUB_LOCATION, 90.0, new Pose2d(Inches.of(469.11).in(Meters), 0.0, new Rotation2d())),
    Arguments.of(RobotConstants.TURRET.RED_HUB_LOCATION, -180.0, new Pose2d(0.0, Inches.of(158.845).in(Meters), new Rotation2d(Degrees.of(180.0).in(Radians)))),
    Arguments.of(RobotConstants.TURRET.RED_HUB_LOCATION, -90.0, new Pose2d(Inches.of(469.11).in(Meters), Inches.of(158.845).in(Meters) * 2, new Rotation2d()))
  );
  }

 
}

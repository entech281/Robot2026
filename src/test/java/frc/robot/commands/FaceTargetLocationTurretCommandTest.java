package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Basic unit tests for {@link FaceTargetLocationTurretCommand}.
 *
 * These tests illustrate the testing framework and how to mock/replace
 * the turret subsystem and the odometry pose provided by RobotIO.
 */
public class FaceTargetLocationTurretCommandTest {

  // tiny tolerance for floating point comparisons
  private static final double EPS = 1e-6;

  @BeforeEach
  public void setup() {
    // reset odometry pose to a known value before each test
    RobotIO.getInstance().updateOdometryPose(new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  // A simple fake turret subsystem that captures the last input passed to updateInputs
  private static class FakeTurretSubsystem extends TurretSubsystem {
    public TurretInput lastInput;

    @Override
    public void updateInputs(TurretInput input) {
      this.lastInput = input;
    }
  }

  @Test
  public void test_executes_updatesRequestedPosition() {
    FakeTurretSubsystem fake = new FakeTurretSubsystem();

    // robot at origin facing 0 degrees
    RobotIO.getInstance().updateOdometryPose(new Pose2d(0.0, 0.0, new Rotation2d()));

    // target is at +X (straight ahead), so angle to target = 0 degrees
    Pose2d target = new Pose2d(1.0, 0.0, new Rotation2d());

    FaceTargetLocationTurretCommand cmd = new FaceTargetLocationTurretCommand(fake, target);
    cmd.execute();

    assertNotNull(fake.lastInput, "updateInputs should have been called with a TurretInput");
  }

  @Test
  public void test_0_angle_to_target_with_different_robot_angle() {
    FakeTurretSubsystem fake = new FakeTurretSubsystem();

    // robot at origin but rotated 90 degrees
    RobotIO.getInstance().updateOdometryPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0)));

    // target is at +X relative to field; angleToTarget = 0, minus robot rotation -> -90
    Pose2d target = new Pose2d(1.0, 0.0, new Rotation2d());

    FaceTargetLocationTurretCommand cmd = new FaceTargetLocationTurretCommand(fake, target);
    cmd.execute();

    assertNotNull(fake.lastInput);
    assertEquals(-90.0, fake.lastInput.getRequestedPosition(), EPS);
  }

  @Test
  public void test_isFinished_returns_false() {
    FakeTurretSubsystem fake = new FakeTurretSubsystem();
    Pose2d target = new Pose2d(0.0, 0.0, new Rotation2d());
    FaceTargetLocationTurretCommand cmd = new FaceTargetLocationTurretCommand(fake, target);
    assertFalse(cmd.isFinished());
  }
}

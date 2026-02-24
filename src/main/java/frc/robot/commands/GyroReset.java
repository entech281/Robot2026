package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.entech.commands.EntechCommand;
import frc.robot.processors.OdometryProcessor;
import frc.robot.sensors.gyro.GyroSensor;

public class GyroReset extends EntechCommand {
  private final Runnable reset;
  private final Runnable correctOdomtry;

  public GyroReset(GyroSensor gyro, OdometryProcessor odometry) {
    reset = () -> {
      gyro.setAngleAdjustment(Angle.ofBaseUnits(0.0, Degrees));
      gyro.zeroYaw();
    };
    correctOdomtry = () -> odometry.resetOdometry(
        new Pose2d(odometry.getEstimatedPose().getTranslation(), Rotation2d.fromDegrees(0)));
  }

  @Override
  public void initialize() {
    reset.run();
    correctOdomtry.run();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

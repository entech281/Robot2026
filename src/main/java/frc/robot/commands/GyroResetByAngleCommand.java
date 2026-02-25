package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.entech.commands.AutonomousException;
import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.processors.OdometryProcessor;
import frc.robot.sensors.gyro.GyroSensor;

public class GyroResetByAngleCommand extends EntechCommand {
  private final Runnable reset;
  private final Runnable set;
  private final Runnable correctOdometry;
  private final double angle;

  public GyroResetByAngleCommand(GyroSensor gyro, OdometryProcessor odometry, String auto) {
    PathPlannerPath startPath;

    try {
      startPath = PathPlannerAuto.getPathGroupFromAutoFile(auto).get(0);
    } catch (Exception e) {
      throw new AutonomousException("Invalid auto file: " + auto, e);
    }

    Optional<Pose2d> startingPose = startPath.getStartingHolonomicPose();
    if (startingPose.isPresent()) {
      angle = startingPose.get().getRotation().getDegrees();
    } else {
      angle = startPath.getStartingDifferentialPose().getRotation().getDegrees();
    }

    reset = gyro::zeroYaw;
    Optional<Alliance> teamOpt = DriverStation.getAlliance();
    if (teamOpt.isPresent()) {
      if (teamOpt.get() == Alliance.Blue) {
        set = () -> gyro
            .setAngleAdjustment(
                RobotIO.getInstance().getGyroOutput().getAngleAdjustment().plus(Angle.ofRelativeUnits(angle, Degrees)));
      } else {
        set = () -> gyro
            .setAngleAdjustment(
                RobotIO.getInstance().getGyroOutput().getAngleAdjustment()
                    .minus(Angle.ofRelativeUnits(angle, Degrees)));
      }
    } else {
      set = () -> gyro
          .setAngleAdjustment(
              RobotIO.getInstance().getGyroOutput().getAngleAdjustment().plus(Angle.ofRelativeUnits(angle, Degrees)));
    }
    correctOdometry = () -> {
      Pose2d pose = new Pose2d(odometry.getEstimatedPose().getTranslation(), Rotation2d.fromDegrees(angle));
      odometry.resetOdometry(pose, Rotation2d.fromDegrees(0.0));
    };
  }

  @Override
  public void initialize() {
    reset.run();
    correctOdometry.run();
    set.run();
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

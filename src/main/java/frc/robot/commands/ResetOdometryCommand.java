package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.processors.OdometryProcessor;

public class ResetOdometryCommand extends EntechCommand {
  private final OdometryProcessor odometry;

  public ResetOdometryCommand(OdometryProcessor odometry) {
    this.odometry = odometry;
  }

  @Override
  public void initialize() {
    Optional<Alliance> team = DriverStation.getAlliance();
    if (team.isPresent() && team.get() == Alliance.Blue) {
      odometry.resetOdometry(RobotConstants.ODOMETRY.INITIAL_POSE);
      return;
    }
    odometry.resetOdometry(RobotConstants.ODOMETRY.INITIAL_POSE);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Moves the turret by a small step while the command is active.
 * Useful for tuning: hold the button to nudge the turret left/right.
 */
public class TurretJogCommand extends EntechCommand {
  private final TurretSubsystem turret;
  private final double stepDegrees;

  /**
   * @param turret turret subsystem
   * @param stepDegrees positive to move toward + degrees, negative for -
   */
  public TurretJogCommand(TurretSubsystem turret, double stepDegrees) {
    super(turret);
    this.turret = turret;
    this.stepDegrees = stepDegrees;
  }

  @Override
  public void initialize() {
    // no-op
  }

  @Override
  public void execute() {
    TurretInput in = new TurretInput();
    double current = RobotIO.getInstance().getTurretOutput().getCurrentPosition();
    in.setRequestedPosition(current + stepDegrees);
    turret.updateInputs(in);
  }

  @Override
  public void end(boolean interrupted) {
    turret.updateInputs(new TurretInput());
  }

  @Override
  public boolean isFinished() {
    return false; // run while held
  }
}

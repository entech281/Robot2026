package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class NudgeTurretCommand extends EntechCommand{
    private final TurretInput turretInput = new TurretInput();
    private final TurretSubsystem turretSS;
    private boolean direction;
    private double position;

    /**
     * 
     * @param turretSubsystem
     * @param direction true for positive
     */
    public NudgeTurretCommand(TurretSubsystem turretSubsystem, boolean direction) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (direction) {
            position = turretSS.getOutputs().getCurrentPosition() + LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount");
        } else {
            position = turretSS.getOutputs().getCurrentPosition() - LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount");
        }
        turretInput.setRequestedPosition(position);
        turretSS.updateInputs(turretInput);
    }

    @Override
    public void execute() {
      turretInput.setRequestedPosition(position);
      turretSS.updateInputs(turretInput);
    }

    @Override
  public void end(boolean interrupted) {
    //Code stops on it's own so nothing to put in the end method
  }

  @Override
  public boolean isFinished() {
    return RobotIO.getInstance().getTurretOutput().isAtRequestedPosition();
  }
}
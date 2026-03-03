package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

public class ManualHoodCommand extends EntechCommand{
    private final HoodInput turretInput = new HoodInput();
    private final HoodSubsystem hoodSS;
    private double position;

    public ManualHoodCommand(HoodSubsystem hoodSubsystem, double position) {
        super(hoodSubsystem);
        this.hoodSS = hoodSubsystem;
        this.position = position;
    }

    @Override
    public void initialize() {
        turretInput.setRequestedPosition(position);
        hoodSS.updateInputs(turretInput);
    }

    @Override
    public void execute() {
        hoodSS.updateInputs(turretInput);
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
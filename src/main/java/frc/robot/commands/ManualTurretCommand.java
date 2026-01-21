package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ManualTurretCommand extends EntechCommand{
    private final TurretInput turretInput = new TurretInput();
    private final TurretSubsystem turretSS;
    private double speed;

    public ManualTurretCommand(TurretSubsystem turretSubsystem, double speed) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        turretInput.setRequestedSpeed(speed);
    }

    @Override
    public void execute() {
        turretSS.updateInputs(turretInput);
    }

    @Override
  public void end(boolean interrupted) {
    //Code stops on it's own so nothing to put in the end method
  }

  @Override
  public boolean isFinished() {
    return RobotIO.getInstance().getTurretOutput().isAtRequestedSpeed();
  }
}
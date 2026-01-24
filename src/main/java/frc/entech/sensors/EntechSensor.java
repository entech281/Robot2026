package frc.entech.sensors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.entech.subsystems.SubsystemOutput;


public abstract class EntechSensor<R extends SubsystemOutput> extends SubsystemBase {
  protected EntechSensor() {}

  public abstract void initialize();

  public abstract boolean isEnabled();

  public abstract Command getTestCommand();

  public abstract R toOutputs();

  public R getOutputs() {
    R out = toOutputs();
    out.setCurrentCommand(this.getClass().getSimpleName() + "");
    out.setDefaultCommand("N/A");
    return out;
  }
}
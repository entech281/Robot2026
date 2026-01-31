package frc.entech.sensors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class EntechSensor<R extends SensorOutput> extends SubsystemBase {
  protected EntechSensor() {
  }

  public abstract void initialize();

  public abstract boolean isEnabled();

  public abstract Command getTestCommand();

  protected abstract R toOutputs();

  public R getOutputs() {
    R out = toOutputs();
    return out;
  }
}
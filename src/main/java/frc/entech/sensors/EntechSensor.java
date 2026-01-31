package frc.entech.sensors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.TestableHardwareI;

public abstract class EntechSensor<R extends SensorOutput> implements TestableHardwareI {
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
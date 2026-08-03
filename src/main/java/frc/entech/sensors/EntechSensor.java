package frc.entech.sensors;

import frc.entech.TestableHardwareI;

public abstract class EntechSensor<R extends SensorOutput> implements TestableHardwareI {
  protected EntechSensor() {
  }

  public abstract void initialize();

  protected abstract R toOutputs();

  public R getOutputs() {
    return toOutputs();
  }
}
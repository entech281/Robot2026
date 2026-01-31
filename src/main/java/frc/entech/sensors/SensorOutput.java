package frc.entech.sensors;

public abstract class SensorOutput {
  public void log() {
    toLog();
  }

  protected abstract void toLog();

  public String getLogName(String val) {
    return getClass().getSimpleName() + "/" + val;
  }
}

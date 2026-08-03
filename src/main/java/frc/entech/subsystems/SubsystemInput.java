package frc.entech.subsystems;

public abstract class SubsystemInput {
  public void log() {
    toLog();
  }

  protected abstract void toLog();
}

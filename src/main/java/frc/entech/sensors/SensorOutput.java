package frc.entech.sensors;

import org.littletonrobotics.junction.Logger;

public abstract class SensorOutput {
  private String currentCommand;
  private String defaultCommand;

  protected void log() {
    Logger.recordOutput(getClass().getSimpleName() + "/CurrentCommand", currentCommand);
    Logger.recordOutput(getClass().getSimpleName() + "/defaultCommand", defaultCommand);
    toLog();
  }

  public abstract void toLog();

  public String getLogName(String val) {
    return getClass().getSimpleName() + "/" + val;
  }

  public String getCurrentCommand() {
    return this.currentCommand;
  }

  public void setCurrentCommand(String currentCommand) {
    this.currentCommand = currentCommand;
  }

  public String getDefaultCommand() {
    return this.defaultCommand;
  }

  public void setDefaultCommand(String defaultCommand) {
    this.defaultCommand = defaultCommand;
  }
}

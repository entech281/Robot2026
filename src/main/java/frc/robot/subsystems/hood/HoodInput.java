package frc.robot.subsystems.hood;

import frc.entech.subsystems.SubsystemInput;
import frc.robot.Logger;
import frc.robot.RobotConstants;

public class HoodInput extends SubsystemInput {
  private boolean activate = true;
  private double requestedPosition = RobotConstants.HOOD.INITIAL_POSITION_DEGREES;

  @Override
  public void toLog() {
    Logger.recordOutput("HoodInput/Activate", activate);
    Logger.recordOutput("HoodInput/RequestedPosition", requestedPosition);
  }

  public boolean getActivate() {
    return this.activate;
  }

  public void setActivate(boolean activate) {
    this.activate = activate;
  }

  public double getRequestedPosition() {
    return this.requestedPosition;
  }

  public void setRequestedPosition(double requestedPosition) {
    this.requestedPosition = requestedPosition;
  }
}

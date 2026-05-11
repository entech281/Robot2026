package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import frc.entech.subsystems.SubsystemInput;
import frc.robot.Logger;
import frc.robot.RobotConstants;

public class TurretInput extends SubsystemInput {
  private boolean activate = true;
  private Angle requestedPosition = RobotConstants.TURRET.HOME_POSITION_DEGREES;

  @Override
  public void toLog() {
    Logger.recordOutput("TurretInput/Activate", activate);
    Logger.recordOutput("TurretInput/RequestedPosition", requestedPosition);
  }

  public boolean getActivate() {
    return this.activate;
  }

  public void setActivate(boolean activate) {
    this.activate = activate;
  }

  public Angle getRequestedPosition() {
    return this.requestedPosition;
  }

  public void setRequestedPosition(Angle requestedPosition) {
    this.requestedPosition = requestedPosition;
  }
}

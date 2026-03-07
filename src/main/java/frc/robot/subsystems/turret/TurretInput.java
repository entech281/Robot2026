package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.units.measure.Angle;
import frc.entech.subsystems.SubsystemInput;
import frc.robot.RobotConstants;

public class TurretInput implements SubsystemInput {
  private boolean activate = true;

  private Angle requestedPosition = RobotConstants.TURRET.HOME_POSITION_DEGREES;

  @Override
  public void toLog(LogTable table) {
    table.put("Activate", activate);
    table.put("Requested position", requestedPosition);
  }

  @Override
  public void fromLog(LogTable table) {
    activate = table.get("Activate", activate);
    requestedPosition = table.get("Requested position", Degrees.of(0.0));
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

package frc.robot.sensors.gyro;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;

public class GyroSensor extends EntechSensor<GyroOutput> {
  private static final boolean ENABLED = true;
  private GyroI gyro;

  @Override
  protected GyroOutput toOutputs() {
    GyroOutput output;

    if (ENABLED) {
      output = gyro.getOutput();
    } else {
      output = new GyroOutput();
    }

    return output;
  }

  @Override
  public String getName() {
    return "GyroSensor";
  }

  @Override
  public void initialize() {
    if (ENABLED) {
      gyro = null;
    }
  }

  @Override
  public boolean isEnabled() {
    return ENABLED;
  }

  public void zeroYaw() {
    if (ENABLED) {
      gyro.zeroYaw();
    }
  }

  @Override
  public Command getTestCommand() {
    return Commands.none();
  }

  public void setAngleAdjustment(Angle angleAdjustment) {
    if (ENABLED) {
      gyro.setAngleOffset(angleAdjustment);
    }
  }
}

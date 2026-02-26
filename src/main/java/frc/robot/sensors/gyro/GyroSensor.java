package frc.robot.sensors.gyro;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

public class GyroSensor extends EntechSensor<GyroOutput> {
  private static final boolean ENABLED = true;
  private GyroI gyro;

  public enum GyroHardware {
    ADIS16448,
    NAVX3
  }

  @Override
  protected GyroOutput toOutputs() {
    GyroOutput output;

    if (ENABLED) {
      gyro.logUniqueData();
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
      switch (RobotConstants.GYRO_HARDWARE) {
        case ADIS16448:
          gyro = new ADIS16448();
          break;
        case NAVX3:
          gyro = new NavX3(0);
          break;
        default:
          gyro = new ADIS16448();
          break;
      }
      gyro.initialize();
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

package frc.robot.sensors.navx;

import frc.entech.NavX.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.entech.util.StoppingCounter;

public class NavXSensor extends EntechSensor<NavXOutput> {
  private static final boolean ENABLED = true;
  private ADIS16448_IMU gyro;
  private final StoppingCounter faultCounter = new StoppingCounter(3.5);
  private boolean faultDetected = false;

  @Override
  protected NavXOutput toOutputs() {
    NavXOutput output = new NavXOutput();

    if (ENABLED) {
      output.setYaw(gyro.getAngle());
      output.setPitch(gyro.getGyroAngleY());
      output.setRoll(gyro.getGyroAngleX());
      output.setYawRate(gyro.getRate());
      output.setChassisSpeeds(getChassisSpeeds());
      output.setZVelocity(0);
      output.setTemperature(gyro.getTemperature());
      output.setAngleAdjustment(0);
      output.setCompassHeading(gyro.getMagneticFieldZ());
      output.setIsCalibrating(false);
      output.setIsMagneticDisturbance(false);
      output.setIsMagnetometerCalibrated(false);
      output.setIsMoving(false);
      output.setIsRotating(false);
      output.setIsFaultDetected(faultDetected);
    }

    if (ENABLED) {
      SmartDashboard.putData(gyro);
      faultDetected = faultCounter.isFinished(gyro.isConnected());
    }

    return output;
  }

  @Override
  public String getName() {
    return "NavXSensor";
  }

  @Override
  public void initialize() {
    if (ENABLED) {
      gyro = new ADIS16448_IMU();

      gyro.calibrate();

      // gyro.reset();
    }
  }

  private ChassisSpeeds getChassisSpeeds() {
    if (ENABLED) {
      double radiansPerSecond = Units.degreesToRadians(gyro.getRate());
      return ChassisSpeeds.fromRobotRelativeSpeeds(0, 0,
          radiansPerSecond, new Rotation2d(gyro.getAngle()));
    } else {
      return ChassisSpeeds.fromRobotRelativeSpeeds(0.0, 0.0, 0.0, new Rotation2d(0.0));
    }
  }

  @Override
  public boolean isEnabled() {
    return ENABLED;
  }

  // @Override
  // public void updateInputs(NavXInput input) {
  // throw new UnsupportedOperation();
  // }

  public void zeroYaw() {
    if (ENABLED) {
      gyro.reset();
    }
  }

  @Override
  public Command getTestCommand() {
    return Commands.none();
  }

  public void setAngleAdjustment(double angleAdjustment) {
    if (ENABLED) {
      // gyro.setAngleAdjustment(angleAdjustment);
    }
  }
}

package frc.robot.sensors.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kelvin;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.entech.sensors.SensorOutput;

public class GyroOutput extends SensorOutput {
  private Angle yaw = Angle.ofRelativeUnits(0, Degrees);
  private Angle pitch = Angle.ofRelativeUnits(0, Degrees);
  private Angle roll = Angle.ofRelativeUnits(0, Degrees);
  private AngularVelocity yawRate = AngularVelocity.ofRelativeUnits(0.0, DegreesPerSecond);
  private Temperature temperature = Temperature.ofRelativeUnits(0, Kelvin);
  private Angle angleAdjustment = Angle.ofRelativeUnits(0, Degrees);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  @Override
  public void toLog() {
    Logger.recordOutput("GyroOutput/yaw", yaw.in(Degrees));
    Logger.recordOutput("GyroOutput/pitch", pitch);
    Logger.recordOutput("GyroOutput/roll", roll);
    Logger.recordOutput("GyroOutput/yawRate", yawRate);
    Logger.recordOutput("GyroOutput/temperature", temperature);
    Logger.recordOutput("GyroOutput/angleAdjustment", angleAdjustment);
    Logger.recordOutput("GyroOutput/chassisSpeeds", chassisSpeeds);
  }

  /**
   * @return Angle return the yaw
   */
  public Angle getYaw() {
    return yaw;
  }

  /**
   * @param yaw the yaw to set
   */
  public void setYaw(Angle yaw) {
    this.yaw = yaw;
  }

  /**
   * @return Angle return the pitch
   */
  public Angle getPitch() {
    return pitch;
  }

  /**
   * @param pitch the pitch to set
   */
  public void setPitch(Angle pitch) {
    this.pitch = pitch;
  }

  /**
   * @return Angle return the roll
   */
  public Angle getRoll() {
    return roll;
  }

  /**
   * @param roll the roll to set
   */
  public void setRoll(Angle roll) {
    this.roll = roll;
  }

  /**
   * @return AngularVelocity return the yawRate
   */
  public AngularVelocity getYawRate() {
    return yawRate;
  }

  /**
   * @param yawRate the yawRate to set
   */
  public void setYawRate(AngularVelocity yawRate) {
    this.yawRate = yawRate;
  }

  /**
   * @return Temperature return the temperature
   */
  public Temperature getTemperature() {
    return temperature;
  }

  /**
   * @param temperature the temperature to set
   */
  public void setTemperature(Temperature temperature) {
    this.temperature = temperature;
  }

  /**
   * @return Angle return the angleAdjustment
   */
  public Angle getAngleAdjustment() {
    return angleAdjustment;
  }

  /**
   * @param angleAdjustment the angleAdjustment to set
   */
  public void setAngleAdjustment(Angle angleAdjustment) {
    this.angleAdjustment = angleAdjustment;
  }

  /**
   * @return ChassisSpeeds return the chassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * @param chassisSpeeds the chassisSpeeds to set
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }
}

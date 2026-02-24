package frc.robot.sensors.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.entech.sensors.SensorOutput;

public class GyroOutput extends SensorOutput {
  private Angle yaw;
  private Angle pitch;
  private Angle roll;
  private AngularVelocity yawRate;
  private Temperature temperature;
  private Angle angleAdjustment;

  private ChassisSpeeds chassisSpeeds;

  @Override
  public void toLog() {
    Logger.recordOutput("GyroOutput/yaw", yaw);
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

package frc.entech.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;

public class SparkMaxOutput {

  private SparkMaxOutput() {
  }

  private double currentSpeed;
  private double appliedOutput;
  private double busVoltage;
  private double motorTemperature;
  private double outputCurrent;
  private boolean hasActiveFault;
  private boolean hasActiveWarning;
  private boolean isFollower;
  private boolean isUpperLimitPressed;
  private boolean isLowerLimitPressed;
  private double currentPosition;

  public static SparkMaxOutput createOutput(SparkBase sm) {
    SparkMaxOutput smo = new SparkMaxOutput();
    smo.currentSpeed = sm.getEncoder().getVelocity();
    smo.busVoltage = sm.getBusVoltage();
    smo.motorTemperature = sm.getMotorTemperature();
    smo.outputCurrent = sm.getOutputCurrent();
    smo.hasActiveFault = sm.hasActiveFault();
    smo.hasActiveWarning = sm.hasStickyFault();
    smo.isFollower = sm.isFollower();
    smo.appliedOutput = sm.getAppliedOutput();
    smo.isUpperLimitPressed = sm.getForwardLimitSwitch().isPressed();
    smo.isLowerLimitPressed = sm.getReverseLimitSwitch().isPressed();
    smo.currentPosition = sm.getEncoder().getPosition();
    return smo;
  }

  public void log(String baseName) {
    Logger.recordOutput(baseName + "/currentSpeed", currentSpeed);
    Logger.recordOutput(baseName + "/busVoltage", busVoltage);
    Logger.recordOutput(baseName + "/motorTemperature", motorTemperature);
    Logger.recordOutput(baseName + "/outputCurrent", outputCurrent);
    Logger.recordOutput(baseName + "/hasActiveFault", hasActiveFault);
    Logger.recordOutput(baseName + "/hasActiveWarning", hasActiveWarning);
    Logger.recordOutput(baseName + "/isFollower", isFollower);
    Logger.recordOutput(baseName + "/appliedOutput", appliedOutput);
    Logger.recordOutput(baseName + "/isUpperLimitPressed", isUpperLimitPressed);
    Logger.recordOutput(baseName + "/isLowerLimitPressed", isLowerLimitPressed);
    Logger.recordOutput(baseName + "/currentPosition", currentPosition);
  }

  /**
   * @return double return the currentSpeed
   */
  public double getCurrentSpeed() {
    return currentSpeed;
  }

  /**
   * @param currentSpeed the currentSpeed to set
   */
  public void setCurrentSpeed(double currentSpeed) {
    this.currentSpeed = currentSpeed;
  }

  /**
   * @param appliedOutput the appliedOutput to set
   */
  public void setAppliedOutput(double appliedOutput) {
    this.appliedOutput = appliedOutput;
  }

  /**
   * @param busVoltage the busVoltage to set
   */
  public void setBusVoltage(double busVoltage) {
    this.busVoltage = busVoltage;
  }

  /**
   * @param motorTemperature the motorTemperature to set
   */
  public void setMotorTemperature(double motorTemperature) {
    this.motorTemperature = motorTemperature;
  }

  /**
   * @param outputCurrent the outputCurrent to set
   */
  public void setOutputCurrent(double outputCurrent) {
    this.outputCurrent = outputCurrent;
  }

  /**
   * @return boolean return the hasActiveFault
   */
  public boolean isHasActiveFault() {
    return hasActiveFault;
  }

  /**
   * @param hasActiveFault the hasActiveFault to set
   */
  public void setHasActiveFault(boolean hasActiveFault) {
    this.hasActiveFault = hasActiveFault;
  }

  /**
   * @return boolean return the hasActiveWarning
   */
  public boolean isHasActiveWarning() {
    return hasActiveWarning;
  }

  /**
   * @param hasActiveWarning the hasActiveWarning to set
   */
  public void setHasActiveWarning(boolean hasActiveWarning) {
    this.hasActiveWarning = hasActiveWarning;
  }

  /**
   * @return boolean return the isFollower
   */
  public boolean isIsFollower() {
    return isFollower;
  }

  /**
   * @param isFollower the isFollower to set
   */
  public void setIsFollower(boolean isFollower) {
    this.isFollower = isFollower;
  }

  /**
   * @return boolean return the isUpperLimitPressed
   */
  public boolean isIsUpperLimitPressed() {
    return isUpperLimitPressed;
  }

  /**
   * @param isUpperLimitPressed the isUpperLimitPressed to set
   */
  public void setIsUpperLimitPressed(boolean isUpperLimitPressed) {
    this.isUpperLimitPressed = isUpperLimitPressed;
  }

  /**
   * @return boolean return the isLowerLimitPressed
   */
  public boolean isIsLowerLimitPressed() {
    return isLowerLimitPressed;
  }

  /**
   * @param isLowerLimitPressed the isLowerLimitPressed to set
   */
  public void setIsLowerLimitPressed(boolean isLowerLimitPressed) {
    this.isLowerLimitPressed = isLowerLimitPressed;
  }

  /**
   * @return double return the currentPosition
   */
  public double getCurrentPosition() {
    return currentPosition;
  }

  /**
   * @param currentPosition the currentPosition to set
   */
  public void setCurrentPosition(double currentPosition) {
    this.currentPosition = currentPosition;
  }

}

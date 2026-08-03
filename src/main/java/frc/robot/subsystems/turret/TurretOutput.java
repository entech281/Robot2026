package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.Logger;

import edu.wpi.first.units.measure.Angle;
import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class TurretOutput extends SubsystemOutput {
  private boolean moving = false;
  private boolean isAtRequestedPosition = false;
  private Angle requestedPosition = Degrees.of(0.0);
  // TODO: match with physical appearance
  private boolean isAtForwardLimitStall = false;
  private boolean isAtReverseLimitStall = false;
  private Angle currentPosition = Degrees.of(0.0);
  private boolean isStalled = false;
  private boolean isPastSofterLowerLimit = false;
  private boolean isPastSofterUpperLimit = false;
  private boolean homingSwitchState = false;
  private boolean homed = false;
  private boolean farLimitSwitchState = false;

  private SparkOutput turretMotor;

  @Override
  public void toLog() {
    Logger.recordOutput("TurretOutput/moving", moving);
    Logger.recordOutput("TurretOutput/requestedPosition", requestedPosition);
    Logger.recordOutput("TurretOutput/currentPosition", currentPosition);
    Logger.recordOutput("TurretOutput/isAtForwardLimitStall", isAtForwardLimitStall);
    Logger.recordOutput("TurretOutput/isAtReverseLimitStall", isAtReverseLimitStall);
    Logger.recordOutput("TurretOutput/isAtRequestedPosition", isAtRequestedPosition);
    Logger.recordOutput("TurretOutput/homingSwitch", homingSwitchState);
    Logger.recordOutput("TurretOutput/homed", homed);
    Logger.recordOutput("TurretOutput/farLimitSwitch", farLimitSwitchState);

    if (turretMotor != null)
      turretMotor.log("TurretOutput/turretMotor");
  }

  public boolean isMoving() {
    return this.moving;
  }

  public void setMoving(boolean moving) {
    this.moving = moving;
  }

  public boolean isAtRequestedPosition() {
    return this.isAtRequestedPosition;
  }

  public void setAtRequestedPosition(boolean isAtRequestedPosition) {
    this.isAtRequestedPosition = isAtRequestedPosition;
  }

  public Angle getCurrentPosition() {
    return this.currentPosition;
  }

  public void setCurrentPosition(Angle currentPosition) {
    this.currentPosition = currentPosition;
  }

  public boolean isAtForwardLimitStall() {
    return this.isAtForwardLimitStall;
  }

  public void setAtForwardLimitStall(boolean isAtForwardLimitStall) {
    this.isAtForwardLimitStall = isAtForwardLimitStall;
  }

  public boolean isAtReverseLimitStall() {
    return this.isAtReverseLimitStall;
  }

  public void setAtReverseLimitStall(boolean isAtReverseLimitStall) {
    this.isAtReverseLimitStall = isAtReverseLimitStall;
  }

  public Angle getRequestedPosition() {
    return this.requestedPosition;
  }

  public void setRequestedPosition(Angle requestedPosition) {
    this.requestedPosition = requestedPosition;
  }

  public SparkOutput getTurretMotor() {
    return this.turretMotor;
  }

  public void setTurretMotor(SparkOutput turretMotor) {
    this.turretMotor = turretMotor;
  }

  public boolean isStalled() {
    return this.isStalled;
  }

  public void setIsStalled(boolean isStalled) {
    this.isStalled = isStalled;
  }

  public boolean isPastSofterLowerLimit() {
    return isPastSofterLowerLimit;
  }

  public void setPastSofterLowerLimit(boolean isPastSofterLowerLimit) {
    this.isPastSofterLowerLimit = isPastSofterLowerLimit;
  }

  public boolean isPastSofterUpperLimit() {
    return isPastSofterUpperLimit;
  }

  public void setPastSofterUpperLimit(boolean isPastSofterUpperLimit) {
    this.isPastSofterUpperLimit = isPastSofterUpperLimit;
  }

  /**
   * @return boolean return the isAtRequestedPosition
   */
  public boolean isIsAtRequestedPosition() {
    return isAtRequestedPosition;
  }

  /**
   * @param isAtRequestedPosition the isAtRequestedPosition to set
   */
  public void setIsAtRequestedPosition(boolean isAtRequestedPosition) {
    this.isAtRequestedPosition = isAtRequestedPosition;
  }

  /**
   * @return boolean return the isAtForwardLimitStall
   */
  public boolean isIsAtForwardLimitStall() {
    return isAtForwardLimitStall;
  }

  /**
   * @param isAtForwardLimitStall the isAtForwardLimitStall to set
   */
  public void setIsAtForwardLimitStall(boolean isAtForwardLimitStall) {
    this.isAtForwardLimitStall = isAtForwardLimitStall;
  }

  /**
   * @return boolean return the isAtReverseLimitStall
   */
  public boolean isIsAtReverseLimitStall() {
    return isAtReverseLimitStall;
  }

  /**
   * @param isAtReverseLimitStall the isAtReverseLimitStall to set
   */
  public void setIsAtReverseLimitStall(boolean isAtReverseLimitStall) {
    this.isAtReverseLimitStall = isAtReverseLimitStall;
  }

  /**
   * @return boolean return the isStalled
   */
  public boolean isIsStalled() {
    return isStalled;
  }

  /**
   * @return boolean return the isPastSofterLowerLimit
   */
  public boolean isIsPastSofterLowerLimit() {
    return isPastSofterLowerLimit;
  }

  /**
   * @param isPastSofterLowerLimit the isPastSofterLowerLimit to set
   */
  public void setIsPastSofterLowerLimit(boolean isPastSofterLowerLimit) {
    this.isPastSofterLowerLimit = isPastSofterLowerLimit;
  }

  /**
   * @return boolean return the isPastSofterUpperLimit
   */
  public boolean isIsPastSofterUpperLimit() {
    return isPastSofterUpperLimit;
  }

  /**
   * @param isPastSofterUpperLimit the isPastSofterUpperLimit to set
   */
  public void setIsPastSofterUpperLimit(boolean isPastSofterUpperLimit) {
    this.isPastSofterUpperLimit = isPastSofterUpperLimit;
  }

  /**
   * @return boolean return the homingSwitchState
   */
  public boolean isHomingSwitchState() {
    return homingSwitchState;
  }

  /**
   * @param homingSwitchState the homingSwitchState to set
   */
  public void setHomingSwitchState(boolean homingSwitchState) {
    this.homingSwitchState = homingSwitchState;
  }

  /**
   * @return boolean return the homed
   */
  public boolean isHomed() {
    return homed;
  }

  /**
   * @param homed the homed to set
   */
  public void setHomed(boolean homed) {
    this.homed = homed;
  }

  /**
   * @return boolean return the farLimitSwitchState
   */
  public boolean isFarLimitSwitchState() {
    return farLimitSwitchState;
  }

  /**
   * @param farLimitSwitchState the farLimitSwitchState to set
   */
  public void setFarLimitSwitchState(boolean farLimitSwitchState) {
    this.farLimitSwitchState = farLimitSwitchState;
  }

}

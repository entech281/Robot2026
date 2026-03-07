package frc.robot.operation;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;

public class UserPolicy {
  private static final UserPolicy instance = new UserPolicy();

  private boolean twistable = false;
  private boolean aligningToAngle = false;
  private double targetAngle = 0.0;
  private boolean isAutoWon = false;
  private Distance manualShotDistance = Meters.of(4);

  private UserPolicy() {
    Logger.recordOutput("UserPolicy/twistable", twistable);
    Logger.recordOutput("UserPolicy/aligningToAngle", aligningToAngle);
    Logger.recordOutput("UserPolicy/targetAngle", targetAngle);
    Logger.recordOutput("UserPolicy/isAutoWon", isAutoWon);
    Logger.recordOutput("UserPolicy/hubOffset", manualShotDistance.in(Meters));
  }

  public static UserPolicy getInstance() {
    return instance;
  }

  public boolean isTwistable() {
    return this.twistable;
  }

  public void setIsTwistable(boolean twistable) {
    this.twistable = twistable;
    Logger.recordOutput("UserPolicy/twistable", twistable);
  }

  public boolean isAligningToAngle() {
    return this.aligningToAngle;
  }

  public void setAligningToAngle(boolean aligningToAngle) {
    this.aligningToAngle = aligningToAngle;
    Logger.recordOutput("UserPolicy/aligningToAngle", aligningToAngle);
  }

  public double getTargetAngle() {
    return this.targetAngle;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
    Logger.recordOutput("UserPolicy/targetAngle", targetAngle);
  }

public boolean isAutoWon() {
    return this.isAutoWon;
  }

  public void setIsAutoWon(boolean isAutoWon) {
    this.isAutoWon = isAutoWon;
    Logger.recordOutput("UserPolicy/isAutoWon", isAutoWon);
  }

  public Distance getHubOffset() {
    return manualShotDistance;
  }

  public void setHubOffset(Distance manualShotDistance) {
    this.manualShotDistance = manualShotDistance;
  }
}

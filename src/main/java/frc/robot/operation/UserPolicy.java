package frc.robot.operation;

import org.littletonrobotics.junction.Logger;

public class UserPolicy {
  private static final UserPolicy instance = new UserPolicy();

  private boolean twistable = false;
  private boolean aligningToAngle = false;
  private double targetAngle = 0.0;
  private boolean isAutoWon = false;

  private UserPolicy() {
    Logger.recordOutput("UserPolicy/twistable", twistable);
    Logger.recordOutput("UserPolicy/aligningToAngle", aligningToAngle);
    Logger.recordOutput("UserPolicy/targetAngle", targetAngle);
    Logger.recordOutput("UserPolicy/isAutoWon", isAutoWon);
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
}

package frc.robot.subsystems.drive;

import frc.robot.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.entech.subsystems.SubsystemInput;

public class DriveInput extends SubsystemInput {
  private double xSpeed;
  private double ySpeed;
  private double rot;
  private Rotation2d gyroAngle;
  private Pose2d latestOdometryPose;
  private String key = "driveInput";

  public DriveInput() {
  }

  public DriveInput(DriveInput template) {
    xSpeed = template.getXSpeed();
    ySpeed = template.getYSpeed();
    rot = template.getRotation();
    gyroAngle = template.getGyroAngle();
    latestOdometryPose = template.getLatestOdometryPose();
  }

  @Override
  public void toLog() {
    Logger.recordOutput("DriveInput/" + key + "/xSpeed", xSpeed);
    Logger.recordOutput("DriveInput/" + key + "/rot", rot);
    Logger.recordOutput("DriveInput/" + key + "/gyroAngle", gyroAngle);
    Logger.recordOutput("DriveInput/" + key + "/pose", latestOdometryPose);
    Logger.recordOutput("DriveInput/" + key + "/ySpeed", ySpeed);
  }

  public double getXSpeed() {
    return this.xSpeed;
  }

  public void setXSpeed(double xSpeed) {
    this.xSpeed = xSpeed;
  }

  public double getYSpeed() {
    return this.ySpeed;
  }

  public void setYSpeed(double ySpeed) {
    this.ySpeed = ySpeed;
  }

  public double getRotation() {
    return this.rot;
  }

  public void setRotation(double rot) {
    this.rot = rot;
  }

  public Rotation2d getGyroAngle() {
    return this.gyroAngle;
  }

  public void setGyroAngle(Rotation2d gyroAngle) {
    this.gyroAngle = gyroAngle;
  }

  public Pose2d getLatestOdometryPose() {
    return this.latestOdometryPose;
  }

  public void setLatestOdometryPose(Pose2d latestOdometryPose) {
    this.latestOdometryPose = latestOdometryPose;
  }

  public String getKey() {
    return this.key;
  }

  public void setKey(String key) {
    this.key = key;
  }
}

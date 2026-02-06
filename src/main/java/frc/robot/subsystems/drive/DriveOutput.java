package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;
import java.util.List;

public class DriveOutput extends SubsystemOutput {
  private SwerveModulePosition[] modulePositions;
  private double[] rawAbsoluteEncoders;
  private double[] virtualAbsoluteEncoders;
  private SwerveModuleState[] moduleStates;
  private ChassisSpeeds speeds;

  private SparkOutput frontLeftDrive;
  private SparkOutput frontRightDrive;
  private SparkOutput rearLeftDrive;
  private SparkOutput rearRightDrive;
  private SparkOutput frontLeftTurn;
  private SparkOutput frontRightTurn;
  private SparkOutput rearLeftTurn;
  private SparkOutput rearRightTurn;

  private List<double[]> drivePositions;
  private List<double[]> turningPositions;
  private double[] timestamps;

  @Override
  public void toLog() {
    Logger.recordOutput("DriveOutput/modulePositions", modulePositions);
    Logger.recordOutput("DriveOutput/rawAbsoluteEncoders", rawAbsoluteEncoders);
    Logger.recordOutput("DriveOutput/virtualAbsoluteEncoders", virtualAbsoluteEncoders);
    Logger.recordOutput("DriveOutput/moduleStates", moduleStates);
    Logger.recordOutput("DriveOutput/chassisSpeed", speeds);

    Logger.recordOutput("DriveOutput/drivePositions/frontLeft", drivePositions.get(0));
    Logger.recordOutput("DriveOutput/drivePositions/frontRight", drivePositions.get(1));
    Logger.recordOutput("DriveOutput/drivePositions/rearLeft", drivePositions.get(2));
    Logger.recordOutput("DriveOutput/drivePositions/rearRight", drivePositions.get(3));

    Logger.recordOutput("DriveOutput/turnPosition/frontLeft", turningPositions.get(0));
    Logger.recordOutput("DriveOutput/turnPosition/frontRight", turningPositions.get(1));
    Logger.recordOutput("DriveOutput/turnPosition/rearLeft", turningPositions.get(2));
    Logger.recordOutput("DriveOutput/turnPosition/rearRight", turningPositions.get(3));

    Logger.recordOutput("DriveOutput/timeStamps", timestamps);

    frontLeftDrive.log("DriveOutput/frontLeft");
    frontRightDrive.log("DriveOutput/frontRight");
    rearLeftDrive.log("DriveOutput/rearLeft");
    rearRightDrive.log("DriveOutput/rearRight");
    frontRightTurn.log("DriveOutput/frontRightTurn");
    rearLeftTurn.log("DriveOutput/rearLeftTurn");
    rearRightTurn.log("DriveOutput/rearRightTurn");
    frontLeftTurn.log("DriveOutput/frontLeftTurn");
  }

  public SwerveModulePosition[] getModulePositions() {
    return this.modulePositions;
  }

  public void setModulePositions(SwerveModulePosition[] modulePositions) {
    this.modulePositions = modulePositions;
  }

  public double[] getRawAbsoluteEncoders() {
    return this.rawAbsoluteEncoders;
  }

  public void setRawAbsoluteEncoders(double[] rawAbsoluteEncoders) {
    this.rawAbsoluteEncoders = rawAbsoluteEncoders;
  }

  public double[] getVirtualAbsoluteEncoders() {
    return this.virtualAbsoluteEncoders;
  }

  public void setVirtualAbsoluteEncoders(double[] virtualAbsoluteEncoders) {
    this.virtualAbsoluteEncoders = virtualAbsoluteEncoders;
  }

  public SwerveModuleState[] getModuleStates() {
    return this.moduleStates;
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    this.moduleStates = moduleStates;
  }

  public ChassisSpeeds getSpeeds() {
    return this.speeds;
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public SparkOutput getFrontLeftDrive() {
    return frontLeftDrive;
  }

  public void setFrontLeftDrive(SparkOutput frontLeftDrive) {
    this.frontLeftDrive = frontLeftDrive;
  }

  public SparkOutput getFrontRightDrive() {
    return frontRightDrive;
  }

  public void setFrontRightDrive(SparkOutput frontRightDrive) {
    this.frontRightDrive = frontRightDrive;
  }

  public SparkOutput getRearLeftDrive() {
    return rearLeftDrive;
  }

  public void setRearLeftDrive(SparkOutput rearLeftDrive) {
    this.rearLeftDrive = rearLeftDrive;
  }

  public SparkOutput getRearRightDrive() {
    return rearRightDrive;
  }

  public void setRearRightDrive(SparkOutput rearRightDrive) {
    this.rearRightDrive = rearRightDrive;
  }

  public SparkOutput getFrontLeftTurn() {
    return frontLeftTurn;
  }

  public void setFrontLeftTurn(SparkOutput frontLeftTurn) {
    this.frontLeftTurn = frontLeftTurn;
  }

  public SparkOutput getFrontRightTurn() {
    return frontRightTurn;
  }

  public void setFrontRightTurn(SparkOutput frontRightTurn) {
    this.frontRightTurn = frontRightTurn;
  }

  public SparkOutput getRearLeftTurn() {
    return rearLeftTurn;
  }

  public void setRearLeftTurn(SparkOutput rearLeftTurn) {
    this.rearLeftTurn = rearLeftTurn;
  }

  public SparkOutput getRearRightTurn() {
    return rearRightTurn;
  }

  public void setRearRightTurn(SparkOutput rearRightTurn) {
    this.rearRightTurn = rearRightTurn;
  }

  /**
   * @return List<double[]> return the drivePositions
   */
  public List<double[]> getDrivePositions() {
    return drivePositions;
  }

  /**
   * @param drivePositions the drivePositions to set
   */
  public void setDrivePositions(List<double[]> drivePositions) {
    this.drivePositions = drivePositions;
  }

  /**
   * @return List<double[]> return the turningPositions
   */
  public List<double[]> getTurningPositions() {
    return turningPositions;
  }

  /**
   * @param turningPositions the turningPositions to set
   */
  public void setTurningPositions(List<double[]> turningPositions) {
    this.turningPositions = turningPositions;
  }

  /**
   * @return double[] return the timestamps
   */
  public double[] getTimestamps() {
    return timestamps;
  }

  /**
   * @param timestamps the timestamps to set
   */
  public void setTimestamps(double[] timestamps) {
    this.timestamps = timestamps;
  }
}

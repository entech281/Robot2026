package frc.robot.io;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveOutput;
import frc.robot.subsystems.navx.NavXOutput;
import frc.robot.subsystems.shooter.ShooterOutput;

public class RobotIO implements DriveInputSupplier {
  private static final RobotIO instance = new RobotIO();

  public static RobotIO getInstance() {
    return instance;
  }

  public static void processInput(LoggableInputs in) {
    Logger.processInputs(in.getClass().getSimpleName(), in);
  }

  private RobotIO() {
  }

  @Override
  public DriveInput getDriveInput() {
    DriveInput di = new DriveInput();
    di.setGyroAngle(Rotation2d.fromDegrees(RobotIO.getInstance().getNavXOutput().getYaw()));
    di.setLatestOdometryPose(latestOdometryPose);
    di.setKey("initialRaw");
    di.setRotation(0.0);
    di.setXSpeed(0.0);
    di.setYSpeed(0.0);
    processInput(di);
    return di;
  }

  public DriveOutput getDriveOutput() {
    return latestDriveOutput;
  }

  public NavXOutput getNavXOutput() {
    return latestNavXOutput;
  }

  public Pose2d getOdometryPose() {
    return latestOdometryPose;
  }

  public ShooterOutput getShooterOutput() {
    return latestShooterOutput;
  }

  public void updateNavx(NavXOutput no) {
    latestNavXOutput = no;
    no.log();
  }

  public void updateDrive(DriveOutput dro) {
    latestDriveOutput = dro;
    dro.log();
  }

  public void updateShooter(ShooterOutput so) {
    latestShooterOutput = so;
    so.log();
  }

  public void updateOdometryPose(Pose2d pose) {
    latestOdometryPose = pose;
    Logger.recordOutput("OdometryPose", pose);
  }

  private NavXOutput latestNavXOutput;
  private DriveOutput latestDriveOutput;
  private ShooterOutput latestShooterOutput;
  private Pose2d latestOdometryPose = RobotConstants.ODOMETRY.INITIAL_POSE;
}

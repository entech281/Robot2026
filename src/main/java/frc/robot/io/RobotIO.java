package frc.robot.io;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveOutput;
import frc.robot.sensors.gyro.GyroOutput;
import frc.robot.sensors.vision.VisionOutput;


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
    di.setGyroAngle(Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)));
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

  public GyroOutput getGyroOutput() {
    return latestGyroOutput;
  }

  public Pose2d getOdometryPose() {
    return latestOdometryPose;
  }


  public VisionOutput getVisionOutput() {
    return latestVisionOutput;
  }

  public PrototypeOutput getPrototypeOutput() {
    return latestPrototypeOutput;
  }

  public PrototypeOutput2 getPrototypeOutput2() {
    return latestPrototypeOutput2;
  }


  
  public void updateGyro(GyroOutput no) {
    latestGyroOutput = no;
    no.log();
  }

  public void updateDrive(DriveOutput dro) {
    latestDriveOutput = dro;
    dro.log();
  }


  public void updateOdometryPose(Pose2d pose) {
    latestOdometryPose = pose;
    Logger.recordOutput("OdometryPose", pose);
  }

  public void updateVision(VisionOutput vo) {
    latestVisionOutput = vo;
    vo.log();
  }

  public void updatePrototype(PrototypeOutput po) {
    latestPrototypeOutput = po;
    po.log();
  }

  public void updatePrototype2(PrototypeOutput2 po2) {
    latestPrototypeOutput2 = po2;
    po2.log();
  }

  private GyroOutput latestGyroOutput;
  private DriveOutput latestDriveOutput;
  private VisionOutput latestVisionOutput;
  private PrototypeOutput latestPrototypeOutput;
  private PrototypeOutput2 latestPrototypeOutput2;
  private Pose2d latestOdometryPose = RobotConstants.ODOMETRY.INITIAL_POSE;
}

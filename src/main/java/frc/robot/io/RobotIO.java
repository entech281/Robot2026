package frc.robot.io;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveOutput;
import frc.robot.subsystems.turret.TurretOutput;
import frc.robot.subsystems.hood.HoodOutput;
import frc.robot.subsystems.intake.IntakeOutput;
import frc.robot.sensors.gyro.GyroOutput;
import frc.robot.sensors.vision.VisionOutput;
import frc.robot.subsystems.shooter.ShooterOutput;
import frc.robot.subsystems.transfer.TransferOutput;

public class RobotIO implements DriveInputSupplier {
  private static final RobotIO instance = new RobotIO();

  public static RobotIO getInstance() {
    return instance;
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
    di.log();
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

  public ShooterOutput getShooterOutput() {
    return latestShooterOutput;
  }

  public VisionOutput getVisionOutput() {
    return latestVisionOutput;
  }

  public TurretOutput getTurretOutput() {
    return latestTurretOutput;
  }

  public HoodOutput getHoodOutput() {
    return latestHoodOutput;
  }

  public TransferOutput getTransferOutput() {
    return latestTransferOutput;
  }

  public IntakeOutput getIntakeOutput() {
    return latestIntakeOutput;
  }

  public void updateGyro(GyroOutput no) {
    latestGyroOutput = no;
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

  public void updateVision(VisionOutput vo) {
    latestVisionOutput = vo;
    vo.log();
  }

  public void updateIntake(IntakeOutput io) {
    latestIntakeOutput = io;
    io.log();
  }

  public void updateHood(HoodOutput ho) {
    latestHoodOutput = ho;
    ho.log();
  }

  public void updateTransfer(TransferOutput tro) {
    latestTransferOutput = tro;
    tro.log();
  }

  public void updateTurret(TurretOutput to) {
    latestTurretOutput = to;
    to.log();
  }

  private GyroOutput latestGyroOutput;
  private DriveOutput latestDriveOutput;
  private VisionOutput latestVisionOutput;
  private ShooterOutput latestShooterOutput;
  private HoodOutput latestHoodOutput;
  private TransferOutput latestTransferOutput;
  private IntakeOutput latestIntakeOutput;
  private Pose2d latestOdometryPose = RobotConstants.ODOMETRY.INITIAL_POSE;
  private TurretOutput latestTurretOutput;
}

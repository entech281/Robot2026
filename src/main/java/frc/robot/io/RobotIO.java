package frc.robot.io;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.climb.ClimbOutput;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.drive.DriveOutput;
import frc.robot.subsystems.hood.HoodOutput;
import frc.robot.subsystems.hopper.HopperOutput;
import frc.robot.subsystems.intake.IntakeOutput;
import frc.robot.sensors.navx.NavXOutput;
import frc.robot.sensors.vision.VisionOutput;
import frc.robot.subsystems.shooter.ShooterOutput;
import frc.robot.subsystems.transfer.TransferOutput;
import frc.robot.subsystems.turret.TurretOutput;

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

  public VisionOutput getVisionOutput() {
    return latestVisionOutput;
  }

  public ClimbOutput getClimbOutput() {
    return latestClimbOutput;
  }

  public TurretOutput getTurretOutput() {
    return latestTurretOutput;
  }

  public HoodOutput getHoodOutput() {
    return latestHoodOutput;
  }

  public HopperOutput getHopperSubsystem() {
    return latestHopperOutput;
  }

  public TransferOutput getTransferOutput() {
    return latestTransferOutput;
  }

  public IntakeOutput getIntakeOutput() {
    return latestIntakeOutput;
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

  public void updateHopper(HopperOutput hopo) {
    latestHopperOutput = hopo;
    hopo.log();
  }

  public void updateTransfer(TransferOutput tro) {
    latestTransferOutput = tro;
    tro.log();
  }

  public void updateClimb(ClimbOutput co) {
    latestClimbOutput = co;
    co.log();
  }

  public void updateTurret(TurretOutput to) {
    latestTurretOutput = to;
    to.log();
  }

  private NavXOutput latestNavXOutput;
  private DriveOutput latestDriveOutput;
  private ShooterOutput latestShooterOutput;
  private VisionOutput latestVisionOutput;
  private TurretOutput latestTurretOutput;
  private HoodOutput latestHoodOutput;
  private HopperOutput latestHopperOutput;
  private ClimbOutput latestClimbOutput;
  private TransferOutput latestTransferOutput;
  private IntakeOutput latestIntakeOutput;
  private Pose2d latestOdometryPose = RobotConstants.ODOMETRY.INITIAL_POSE;
}

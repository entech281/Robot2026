package frc.robot.operation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandFactory;
import frc.robot.HardwareManager;
import frc.robot.RobotConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.HoodJogCommand;
import frc.robot.commands.ManualHoodCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeVariableCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunTransferCommand;
import frc.robot.commands.ShooterLag;
import frc.robot.commands.TurretContinuousNudgeCommand;
import frc.robot.commands.TurretJogCommand;
import frc.robot.commands.TwistCommand;
import frc.robot.commands.XDriveCommand;
import frc.robot.io.DriveInputSupplier;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.processors.OdometryProcessor;
import frc.robot.subsystems.drive.DriveInput;

public class OperatorInterface
    implements DriveInputSupplier {
  private CommandJoystick joystickController;
  private CommandXboxController xboxController;

  private CommandXboxController tuningController;

  private CommandJoystick operatorPanel;

  private final CommandFactory commandFactory;
  private final HardwareManager subsystemManager;
  private final OdometryProcessor odometry;

  public OperatorInterface(CommandFactory commandFactory, HardwareManager subsystemManager,
      OdometryProcessor odometry) {
    this.commandFactory = commandFactory;
    this.subsystemManager = subsystemManager;
    this.odometry = odometry;
  }

  public void create() {

    xboxController = new CommandXboxController(RobotConstants.PORTS.CONTROLLER.DRIVER_CONTROLLER);
    enableXboxBindings();

    if (DriverControllerUtils
        .controllerIsPresent(RobotConstants.PORTS.CONTROLLER.TUNING_CONTROLLER)) {
      tuningController = new CommandXboxController(RobotConstants.PORTS.CONTROLLER.TUNING_CONTROLLER);
      enableTuningControllerBindings();
    }

    enableTriggers();

    operatorPanel = new CommandJoystick(RobotConstants.PORTS.CONTROLLER.SCORE_PANEL);
    scoreOperatorBindings();
  }

  public void enableTuningControllerBindings() {
    tuningController.a().whileTrue(new RunIntakeCommand(subsystemManager.getIntakeSubsystem()));
    tuningController.b().whileTrue(new RunTransferCommand(subsystemManager.getTransferSubsystem()));
    tuningController.x().whileTrue(new RunShooterCommand(subsystemManager.getShooterSubsystem()));

    tuningController.povLeft()
        .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), 10.0));
    tuningController.povRight()
        .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), -10.0));

    tuningController.povDown().onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), -1.0));

    tuningController.povUp().onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), 1.0));

    tuningController.leftBumper().onTrue(new InstantCommand(
        () -> UserPolicy.getInstance().setShooterRPM(UserPolicy.getInstance().getShooterRPM().minus(RPM.of(100)))));
    tuningController.rightBumper().onTrue(new InstantCommand(
        () -> UserPolicy.getInstance().setShooterRPM(UserPolicy.getInstance().getShooterRPM().plus(RPM.of(100)))));

  }

  public void configureBindings() {
    if (DriverControllerUtils.currentControllerIsXbox()) {
      xboxController = new CommandXboxController(RobotConstants.PORTS.CONTROLLER.DRIVER_CONTROLLER);
      enableXboxBindings();
    } else {
      joystickController = new CommandJoystick(RobotConstants.PORTS.CONTROLLER.DRIVER_CONTROLLER);
      enableJoystickBindings();
    }
  }

  public void enableJoystickBindings() {
    joystickController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_JOYSTICK.TWIST)
        .whileTrue(new TwistCommand());
    joystickController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_JOYSTICK.GYRO_RESET)
        .onTrue(new GyroReset(subsystemManager.getGyroSubsystem(), odometry));

    subsystemManager.getDriveSubsystem()
        .setDefaultCommand(new DriveCommand(subsystemManager.getDriveSubsystem(), this));

    joystickController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_JOYSTICK.RESET_ODOMETRY)
        .onTrue(new ResetOdometryCommand(odometry));
  }

  public void enableXboxBindings() {
    xboxController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_XBOX.GYRO_RESET)
        .onTrue(new GyroReset(subsystemManager.getGyroSubsystem(), odometry));

    subsystemManager.getDriveSubsystem()
        .setDefaultCommand(new DriveCommand(subsystemManager.getDriveSubsystem(), this));

    xboxController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_XBOX.DRIVE_X)
        .whileTrue(new XDriveCommand(subsystemManager.getDriveSubsystem()));

    xboxController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_XBOX.RESET_ODOMETRY)
        .onTrue(new ResetOdometryCommand(odometry));

    xboxController.y().whileTrue(new RepeatCommand(
        new ManualHoodCommand(subsystemManager.getHoodSubsystem(), 0)));

    xboxController.leftBumper().whileTrue(new RepeatCommand(commandFactory.getRotateForBumpCommand()));
    xboxController.rightBumper().whileTrue(new RepeatCommand(commandFactory.getRotateForBumpCommand()));
  }

  public void enableTriggers() {
    new Trigger(() -> RobotIO.getInstance().getTurretOutput().isPastSofterLowerLimit())
        .onTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.5)))
        .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.0)));

    new Trigger(() -> RobotIO.getInstance().getTurretOutput().isPastSofterUpperLimit())
        .onTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kRightRumble, 0.5)))
        .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kRightRumble, 0.0)));

  }

  public void scoreOperatorBindings() {
    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.FIRE).whileTrue(new ParallelCommandGroup(
        new RunShooterCommand(subsystemManager.getShooterSubsystem()), new SequentialCommandGroup(new WaitCommand(2),
            new RunTransferCommand(subsystemManager.getTransferSubsystem()))));
    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.AUTO_FIRE)
        .whileTrue(commandFactory.getFullShootCommand())
        .onFalse(new ShooterLag(subsystemManager.getShooterSubsystem()));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.SNOWBLOW_FIRE)
        .and(xboxController.y().negate())
        .whileTrue(commandFactory.getSnowblowCommand())
        .onFalse(new ShooterLag(subsystemManager.getShooterSubsystem()));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.INTAKE)
        .whileTrue(new RunIntakeVariableCommand(subsystemManager.getIntakeSubsystem()));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.OUTTAKE)
        .whileTrue(new RunIntakeCommand(subsystemManager.getIntakeSubsystem(), false))
        .whileTrue(new RunTransferCommand(subsystemManager.getTransferSubsystem(), false));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.PRESET_1_FIRE)
        .whileTrue(new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
            subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(),
            RobotIO.getInstance().getTurretOutput().getCurrentPosition(),
            RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetOneRPM")),
            Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetOneDegrees")),
            false));
    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.PRESET_2_FIRE)
        .whileTrue(new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
            subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(),
            RobotIO.getInstance().getTurretOutput().getCurrentPosition(),
            RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetTwoRPM")),
            Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetTwoDegrees")), false));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.TURRET_NUDGE_UP)
        .onTrue(new InstantCommand(() -> UserPolicy.getInstance()
            .setShooterCalculatorSpeedMultiplier(
                UserPolicy.getInstance().getShooterCalculatorSpeedMultiplier() + 0.1)));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.TURRET_NUDGE_DOWN)
        .onTrue(new InstantCommand(() -> UserPolicy.getInstance()
            .setShooterCalculatorSpeedMultiplier(
                UserPolicy.getInstance().getShooterCalculatorSpeedMultiplier() - 0.1)));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.CLIMB).onTrue(new RepeatCommand(
        new ManualHoodCommand(subsystemManager.getHoodSubsystem(), 0)));

    operatorPanel.button(12)
        .whileTrue(new TurretContinuousNudgeCommand(subsystemManager.getTurretSubsystem(), true));

    operatorPanel.button(6)
        .whileTrue(new TurretContinuousNudgeCommand(subsystemManager.getTurretSubsystem(), false));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.DISTANCE_DOWN)
        .onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), -5.0));

    operatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.DISTANCE_UP)
        .onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), 5.0));
  }

  /*
   * These force commands to accept inputs, not raw joysticks and stuff also here
   * we log any inputs
   * handed to consumers, so they dont have to
   */

  @Override
  public DriveInput getDriveInput() {
    DriveInput di = new DriveInput();

    di.setGyroAngle(Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)));
    di.setLatestOdometryPose(odometry.getEstimatedPose());
    di.setKey("initialRaw");

    if (DriverControllerUtils.currentControllerIsXbox()) {
      di.setXSpeed(-this.xboxController.getLeftY());
      di.setYSpeed(-this.xboxController.getLeftX());
      di.setRotation(DriverControllerUtils.getXboxRotation(this.xboxController));
    } else if (DriverControllerUtils.controllerIsPresent(RobotConstants.PORTS.CONTROLLER.TEST_JOYSTICK)) {
      di.setXSpeed(-this.joystickController.getY());
      di.setYSpeed(-this.joystickController.getX());
      di.setRotation(-this.joystickController.getZ());
    } else {
      di.setXSpeed(0);
      di.setYSpeed(0);
      di.setRotation(0);
    }

    di.log();
    return di;
  }
}
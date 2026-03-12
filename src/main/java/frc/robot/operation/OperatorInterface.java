package frc.robot.operation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.entech.operatorpanel.OutputJoystick;
import frc.entech.operatorpanel.OutputJoystick.Color;
import frc.entech.operatorpanel.OutputJoystick.LedNumber;
import frc.robot.CommandFactory;
import frc.robot.HardwareManager;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.LiveTuning;
import frc.robot.commands.DeployHopper;
import frc.robot.commands.DriveCommand;
import frc.robot.Robot;
import frc.robot.commands.AimTurretLiveCommand;
import frc.robot.commands.DeployHopper;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DropHopper;
import frc.robot.commands.DropThenRaiseHopper;
import frc.robot.commands.FaceTargetLocationTurretCommand;
import frc.robot.commands.GyroReset;
import frc.robot.commands.HoodJogCommand;
import frc.robot.commands.ManualHoodCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.NudgeTurretCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunShooterAtLiveSpeedCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunTransferCommand;
import frc.robot.commands.TransfreFoo;
import frc.robot.commands.TurretJogCommand;
import frc.robot.commands.TwistCommand;
import frc.robot.io.DebugInput;
import frc.robot.io.DebugInputSupplier;
import frc.robot.io.DriveInputSupplier;
import frc.robot.io.OperatorInput;
import frc.robot.io.OperatorInputSupplier;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.processors.OdometryProcessor;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.util.ShiftStateTracker;
import frc.robot.util.ShiftStateTracker.ShiftState;
import frc.robot.util.ShooterCalculator;

public class OperatorInterface
    implements DriveInputSupplier, DebugInputSupplier, OperatorInputSupplier {
  private CommandJoystick joystickController;
  private CommandXboxController xboxController;

  private CommandXboxController tuningController;

  private CommandJoystick scoreOperatorPanel;
  private CommandJoystick alignOperatorPanel;

  private OutputJoystick shiftLightOutput;

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
    if (DriverControllerUtils.controllerIsPresent(RobotConstants.PORTS.CONTROLLER.TEST_JOYSTICK)) {
      joystickController = new CommandJoystick(RobotConstants.PORTS.CONTROLLER.TEST_JOYSTICK);
      enableJoystickBindings();
    }

    if (DriverControllerUtils
        .controllerIsPresent(RobotConstants.PORTS.CONTROLLER.TUNING_CONTROLLER)) {
      tuningController = new CommandXboxController(RobotConstants.PORTS.CONTROLLER.TUNING_CONTROLLER);
      enableTuningControllerBindings();
    }

    if (DriverControllerUtils
        .controllerIsPresent(RobotConstants.PORTS.CONTROLLER.SHIFT_LIGHT_OUTPUT)) {
      shiftLightOutput = new OutputJoystick(RobotConstants.PORTS.CONTROLLER.SHIFT_LIGHT_OUTPUT);
      enableOperatorOutputBindings();
    }

    enableTriggers();

    scoreOperatorPanel = new CommandJoystick(RobotConstants.PORTS.CONTROLLER.SCORE_PANEL);
    scoreOperatorBindings();

    alignOperatorPanel = new CommandJoystick(RobotConstants.PORTS.CONTROLLER.ALIGN_PANEL);
    alignOperatorBindings();

  }

  public void enableTuningControllerBindings() {
    // Basic motor toggles for quick tuning
    tuningController.a().whileTrue(new RunIntakeCommand(subsystemManager.getIntakeSubsystem()));
    tuningController.b().whileTrue(new RunTransferCommand(subsystemManager.getTransferSubsystem()));
    tuningController.x().whileTrue(new RunShooterCommand(subsystemManager.getShooterSubsystem()));
    // Momentary drop-then-raise hopper cycle for tuning
    tuningController.y().onTrue(new DropThenRaiseHopper(subsystemManager.getHopperSubsystem()));

    // Turret tuning: bumpers jog left/right alrwhile held (small steps)
    tuningController.povLeft()
        .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), -5.0));
    tuningController.povRight()
        .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), 5.0));

    // Hood tuning: use POV (d-pad) up/down to jog hood +/-1 degrees while held
    tuningController.povDown().onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), -1.0));

    tuningController.povUp().onTrue(new HoodJogCommand(subsystemManager.getHoodSubsystem(), 1.0));
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

    // xboxController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_XBOX.DRIVE_X)
    // .whileTrue(new XDriveCommand(subsystemManager.getDriveSubsystem()));

    xboxController.button(RobotConstants.PORTS.CONTROLLER.BUTTONS_XBOX.RESET_ODOMETRY)
        .onTrue(new ResetOdometryCommand(odometry));

    xboxController.a().whileTrue(new ManualTurretCommand(subsystemManager.getTurretSubsystem(),
        0));

    xboxController.b().whileTrue(new ManualTurretCommand(subsystemManager.getTurretSubsystem(),
        30));

    xboxController.y().whileTrue(new ManualTurretCommand(subsystemManager.getTurretSubsystem(),
        -30));

    xboxController.x()
        .whileTrue(new ParallelCommandGroup(
            new RunTransferCommand(subsystemManager.getTransferSubsystem(), false)));

    xboxController.leftBumper().whileTrue(new RepeatCommand(commandFactory.getRotateForBumpCommand()));
    xboxController.rightBumper().whileTrue(new RepeatCommand(commandFactory.getRotateForBumpCommand()));
  }

  public void enableOperatorOutputBindings() {
    // Shift light LED triggers — reads wonAuto live from UserPolicy each cycle
    // No yellow on OutputJoystick so warning states use FAST blink instead
    shiftLightOutput.setLED(LedNumber.k1, Color.GREEN, true);

    new Trigger(() -> getShiftState() == ShiftState.YOUR_SHIFT)
        .onTrue(new InstantCommand(() -> {
          DriverStation.reportWarning("Green Solid", false);
          shiftLightOutput.setLED(LedNumber.k0, Color.GREEN, true);
        }));

    new Trigger(() -> getShiftState() == ShiftState.SHIFT_ENDING)
        .onTrue(new InstantCommand(() -> {
          DriverStation.reportWarning("Green Blinking", false);
          shiftLightOutput.setLED(LedNumber.k0, Color.GREEN, false);
        }));

    new Trigger(() -> getShiftState() == ShiftState.THEIR_SHIFT)
        .onTrue(new InstantCommand(() -> {
          DriverStation.reportWarning("Red Solid", false);
          shiftLightOutput.setLED(LedNumber.k0, Color.RED, true);
        }));

    new Trigger(() -> getShiftState() == ShiftState.SHIFT_STARTING)
        .onTrue(new InstantCommand(() -> {
          DriverStation.reportWarning("Red Blinking", false);
          shiftLightOutput.setLED(LedNumber.k0, Color.RED, false);
        }));
  }

  public void enableTriggers() {
    new Trigger(() -> RobotIO.getInstance().getTurretOutput().isPastSofterLowerLimit())
        .onTrue(new InstantCommand(() -> DriverStation.reportWarning("Turret past softer lower limit!", false)))
        .onTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kLeftRumble, 0.5)));

    new Trigger(() -> RobotIO.getInstance().getTurretOutput().isPastSofterUpperLimit())
        .onTrue(new InstantCommand(() -> DriverStation.reportWarning("Turret past softer upper limit!", false)))
        .onTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kRightRumble, 0.5)));

  }

  private ShiftState getShiftState() {
    ShiftStateTracker liveTracker = new ShiftStateTracker(ShiftStateTracker.areWeFirstAlliance(),
        RobotConstants.LiveTuning.VALUES.get("ShiftStateTracker/WarningSeconds"));
    return liveTracker.getState(DriverStation.getMatchTime());
  }

  public void scoreOperatorBindings() {
    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.FIRE)
        .whileTrue(new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
            subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(),
            subsystemManager.getTurretSubsystem().getOutputs().getCurrentPosition(),
            () -> {

              Angle angle = subsystemManager.getGyroSubsystem().getOutputs().getYaw();

              angle = Degrees.of(angle.in(Degrees) % 360);

              Pose3d currentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
                  .plus(RobotConstants.SHOOTER.SHOT_TRANSFORM);
              Pose3d targetPose = currentPose.plus(
                  new Transform3d(UserPolicy.getInstance().getHubOffset().in(Meters) * Math.sin(angle.in(Radians)),
                      UserPolicy.getInstance().getHubOffset().in(Meters) * Math.cos(angle.in(Radians)), 0.0,
                      new Rotation3d()));

              return new ShooterCalculator(subsystemManager.getDriveSubsystem().getChassisSpeeds(), currentPose,
                  targetPose, Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS), RobotConstants.SHOOTER.MAX_SHOT_SPEED, RobotConstants.SHOOTER.MIN_SHOT_SPEED, RobotConstants.SHOOTER.MAX_SHOT_DISTANCE, RobotConstants.SHOOTER.MIN_SHOT_DISTANCE);
            }))
        .onFalse(commandFactory.getStopShootingCommand());

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.AUTO_FIRE)
        .whileTrue(commandFactory.getFullShootCommand());

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.SNOWBLOW_FIRE)
        .whileTrue(commandFactory.getSnowblowCommand())
        .onFalse(commandFactory.getStopShootingCommand());

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.INTAKE)
        .whileTrue(new RunIntakeCommand(subsystemManager.getIntakeSubsystem(), true));
    // TODO add stop intake for both of these onFalse()
    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.OUTTAKE)
        .whileTrue(new RunIntakeCommand(subsystemManager.getIntakeSubsystem(), false));

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.DEPLOY_HOPPER)
        .onTrue(new DeployHopper(subsystemManager.getHopperSubsystem(), true))
        .onFalse(new DeployHopper(subsystemManager.getHopperSubsystem(), false));

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.PRESET_1_FIRE)
        .whileTrue(commandFactory.getPresetShootCommand(RobotConstants.SHOOTER.SHOT_PRESET_ONE))
        .onFalse(commandFactory.getStopShootingCommand());
    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.PRESET_2_FIRE)
        .whileTrue(commandFactory.getPresetShootCommand(RobotConstants.SHOOTER.SHOT_PRESET_TWO))
        .onFalse(commandFactory.getStopShootingCommand());

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.TURRET_NUDGE_UP)
    .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount")));

    scoreOperatorPanel.button(0)
    .onTrue(new TurretJogCommand(subsystemManager.getTurretSubsystem(), -LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount")));

    // Latching toggle switch — pressed down = won auto, released = did not win auto
    // scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.WON_AUTO_SWITCH)
    // .onTrue(new InstantCommand(() ->
    // UserPolicy.getInstance().setIsAutoWon(true)))
    // .onFalse(new InstantCommand(() ->
    // UserPolicy.getInstance().setIsAutoWon(false)));

    // scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.TURRET_NUDGE_UP)
    //     .onTrue(new NudgeTurretCommand(subsystemManager.getTurretSubsystem(), true))
    //     .onFalse(commandFactory.getStopShootingCommand());
    // scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.TURRET_NUDGE_DOWN)
    //     .onTrue(new NudgeTurretCommand(subsystemManager.getTurretSubsystem(), false))
    //     .onFalse(commandFactory.getStopShootingCommand());

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.DISTANCE_UP)
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> UserPolicy.getInstance()
                    .setHubOffset(UserPolicy.getInstance().getHubOffset().plus(
                        Meters.of(LiveTuningHandler.getInstance().getValue("UserPolicy/DistanceNudgeAmountMeters"))))),
                new InstantCommand(() -> {
                  Angle angle = subsystemManager.getGyroSubsystem().getOutputs().getYaw();

                  angle = Degrees.of(angle.in(Degrees) % 360);

                  Pose3d currentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
                      .plus(RobotConstants.SHOOTER.SHOT_TRANSFORM);
                  Pose3d targetPose = currentPose.plus(
                      new Transform3d(UserPolicy.getInstance().getHubOffset().in(Meters) * Math.sin(angle.in(Radians)),
                          UserPolicy.getInstance().getHubOffset().in(Meters) * Math.cos(angle.in(Radians)), 0.0,
                          new Rotation3d()));

            ShooterCalculator shooterCalculator = new ShooterCalculator(subsystemManager.getDriveSubsystem().getChassisSpeeds(), currentPose, targetPose, Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS), RobotConstants.SHOOTER.MAX_SHOT_SPEED, RobotConstants.SHOOTER.MIN_SHOT_SPEED, RobotConstants.SHOOTER.MAX_SHOT_DISTANCE, RobotConstants.SHOOTER.MIN_SHOT_DISTANCE);

                  CommandScheduler.getInstance().schedule(new ManualHoodCommand(subsystemManager.getHoodSubsystem(),
                      shooterCalculator.calculateShot().getIdealShot().getHoodAngle().in(Degrees)));
                })));

    // scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.INCREASE_DISTANCE_OFFSET)
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> UserPolicy.getInstance()
    //                 .setHubOffset(UserPolicy.getInstance().getHubOffset().minus(
    //                     Meters.of(LiveTuningHandler.getInstance().getValue("UserPolicy/DistanceNudgeAmountMeters"))))),
    //             new InstantCommand(() -> {
    //               Angle angle = subsystemManager.getGyroSubsystem().getOutputs().getYaw();
    //             }
    //             )));

    scoreOperatorPanel.button(RobotConstants.SCORE_OPERATOR_PANEL.BUTTONS.DISTANCE_DOWN)
    .onTrue(
      new SequentialCommandGroup(
        new InstantCommand( () -> UserPolicy.getInstance().setHubOffset(UserPolicy.getInstance().getHubOffset().minus(Meters.of(LiveTuningHandler.getInstance().getValue("UserPolicy/DistanceNudgeAmountMeters"))))),
        new InstantCommand( () -> {
            Angle angle = subsystemManager.getGyroSubsystem().getOutputs().getYaw();

                  Pose3d currentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
                      .plus(RobotConstants.SHOOTER.SHOT_TRANSFORM);
                  Pose3d targetPose = currentPose.plus(
                      new Transform3d(-UserPolicy.getInstance().getHubOffset().in(Meters) * Math.sin(angle.in(Radians)),
                          -UserPolicy.getInstance().getHubOffset().in(Meters) * Math.cos(angle.in(Radians)), 0.0,
                          new Rotation3d()));

                  ShooterCalculator shooterCalculator = new ShooterCalculator(
                      subsystemManager.getDriveSubsystem().getChassisSpeeds(), currentPose, targetPose,
                      Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS), RobotConstants.SHOOTER.MAX_SHOT_SPEED,
                      RobotConstants.SHOOTER.MIN_SHOT_SPEED, RobotConstants.SHOOTER.MAX_SHOT_DISTANCE,
                      RobotConstants.SHOOTER.MIN_SHOT_DISTANCE);

            CommandScheduler.getInstance().schedule(new ManualHoodCommand(subsystemManager.getHoodSubsystem(), shooterCalculator.calculateShot().getIdealShot().getHoodAngle().in(Degrees)));
        })
      ));
  }

  // adding more later
  public void driverShiftWarning() {
    // TODO
    // will be fixed later, just a placeholder for now
  }

  public void alignOperatorBindings() {

    // TODO: Move to CommandFactory
    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // if (alliance.get() == DriverStation.Alliance.Blue) {
    // subsystemManager.getTurretSubsystem().setDefaultCommand(new
    // FaceTargetLocationTurretCommand(
    // subsystemManager.getTurretSubsystem(),
    // RobotConstants.TURRET.BLUE_HUB_LOCATION.toPose2d()));
    // } else if (alliance.get() == DriverStation.Alliance.Red) {
    // subsystemManager.getTurretSubsystem().setDefaultCommand(new
    // FaceTargetLocationTurretCommand(
    // subsystemManager.getTurretSubsystem(),
    // RobotConstants.TURRET.RED_HUB_LOCATION.toPose2d()));
    // }
    // } else {
    // DriverStation.reportWarning("Could not get alliance, TurretSubsystem not set
    // to track by default", false);
    // }
    // subsystemManager.getTurretSubsystem()
    // .setDefaultCommand(new
    // AimTurretLiveCommand(subsystemManager.getTurretSubsystem()));

  }

  /*
   * These force commands to accept inputs, not raw joysticks and stuff also here
   * we log any inputs
   * handed to consumers, so they dont have to
   */
  @Override
  public DebugInput getDebugInput() {
    DebugInput di = new DebugInput();
    RobotIO.processInput(di);
    return di;
  }

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

    RobotIO.processInput(di);
    return di;
  }

  @Override
  public OperatorInput getOperatorInput() {
    OperatorInput oi = new OperatorInput();
    RobotIO.processInput(oi);
    return oi;
  }
}
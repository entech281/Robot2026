package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.entech.TestableHardwareI;
import frc.entech.commands.AutonomousException;
import frc.entech.commands.InstantAnytimeCommand;
import frc.robot.commands.GyroResetByAngleCommand;
import frc.robot.commands.KillDriveCommand;
import frc.robot.commands.ManualHoodCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.FaceTargetLocationTurretCommand;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.commands.ShooterLag;
import frc.robot.commands.VirtualTargetAutoShootCommand;
import frc.robot.commands.XDriveCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.ManualTurretCommandSupplier;
import frc.robot.commands.RunShooterAtLiveSpeedCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunTestCommand;
import frc.robot.commands.RunTransferCommand;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.livetuning.WheelDiameterCharacterizer;
import frc.robot.operation.UserPolicy;
import frc.robot.processors.OdometryProcessor;
import frc.robot.sensors.gyro.GyroSensor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;
import frc.robot.util.TurretCalculator;

@SuppressWarnings("unused")
public class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final GyroSensor gyroSubsystem;
  private final OdometryProcessor odometry;
  private final HardwareManager subsystemManager;
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> testChooser;

  public CommandFactory(HardwareManager subsystemManager, OdometryProcessor odometry) {
    this.driveSubsystem = subsystemManager.getDriveSubsystem();
    this.gyroSubsystem = subsystemManager.getGyroSubsystem();
    this.odometry = odometry;
    this.subsystemManager = subsystemManager;
    // subsystemManager.getShooterSubsystem()
    // .setDefaultCommand(new
    // RunShooterAtLiveSpeedCommand(subsystemManager.getShooterSubsystem()));
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException e) {
      throw new AutonomousException("Failed to load robot config", e);
    } catch (ParseException e) {
      throw new AutonomousException("Failed to parse robot config", e);
    }

    ShuffleboardTab tab = Shuffleboard.getTab("stuffs");
    tab.add("Save", new InstantAnytimeCommand(() -> LiveTuningHandler.getInstance().saveToJSON()));
    tab.add("Load", new InstantAnytimeCommand(() -> LiveTuningHandler.getInstance().resetToJSON()));
    tab.add("Code Defaults", new InstantAnytimeCommand(() -> LiveTuningHandler.getInstance().resetToDefaults()));
    tab.add("Characterize Wheel Diameter", getWheelCharacterizationCommand());
    tab.add("Use Beta Shot Calculation",
        new InstantAnytimeCommand(() -> UserPolicy.getInstance().setUseBeta(!UserPolicy.getInstance().getUseBeta())));
    this.testChooser = getTestCommandChooser();
    testChooser.addOption("All tests", getTestCommand());
    Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, "No Current Test");
    SmartDashboard.putData("Test Chooser", testChooser);
    Shuffleboard.getTab("stuffs").add("Run Test", new RunTestCommand(testChooser));
    tab.add("FOUNTAIN", getFountainCommand());
    tab.add("KILL DRIVE", new KillDriveCommand(subsystemManager.getDriveSubsystem()));

    AutoBuilder.configure(odometry::getEstimatedPose,
        odometry::resetOdometry,
        driveSubsystem::getChassisSpeeds,
        (speeds, feedForwards) -> driveSubsystem.pathFollowDrive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(8.5, 3, 0.1),
            new PIDConstants(RobotConstants.AUTONOMOUS.ROTATION_CONTROLLER_P, 0.0, 0.0)),
        config, () -> {

          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, driveSubsystem);

    NamedCommands.registerCommand("AutoShoot", getFullShootCommand());
    NamedCommands.registerCommand("Intake", new RunIntakeCommand(subsystemManager.getIntakeSubsystem()));
    NamedCommands.registerCommand("ReverseIntake", new RunIntakeCommand(subsystemManager.getIntakeSubsystem(), false));
    NamedCommands.registerCommand("TrenchPreset",
        getPresetShootCommand(RobotConstants.SHOOTER.SHOT_PRESET_TWO));
    NamedCommands.registerCommand("TowerPreset",
        getPresetShootCommand(RobotConstants.SHOOTER.SHOT_PRESET_ONE));
    // NamedCommands.registerCommand("TrenchPreset",
    // Commands.none());
    // NamedCommands.registerCommand("TowerPreset",
    // Commands.none());
    NamedCommands.registerCommand("Snowblow", getSnowblowCommand());
    NamedCommands.registerCommand("X", new XDriveCommand(driveSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutoCommand() {
    SequentialCommandGroup auto = new SequentialCommandGroup();
    auto.addCommands(new GyroResetByAngleCommand(gyroSubsystem, odometry, autoChooser.getSelected().getName()));
    auto.addCommands(new WaitCommand(0.5));
    auto.addCommands(autoChooser.getSelected());
    return auto;
  }

  public Command getTestCommand() {
    SequentialCommandGroup allTests = new SequentialCommandGroup();
    for (TestableHardwareI subsystem : subsystemManager.getSubsystemList()) {
      if (subsystem.isEnabled()) {
        addSubsystemTest(allTests, subsystem);
      }
    }
    allTests.addCommands(Commands.runOnce(() -> Logger
        .recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, "No Current Tests.")));
    return allTests;
  }

  private static void addSubsystemTest(SequentialCommandGroup group,
      TestableHardwareI subsystem) {

    group.addCommands(
        Commands.runOnce(() -> Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
            String.format("%s: Start", subsystem.getName()))),
        subsystem.getTestCommand(),
        Commands.runOnce(() -> Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
            String.format("%s: Finished", subsystem.getName()))));
  }

  private SendableChooser<Command> getTestCommandChooser() {
    SendableChooser<Command> testCommandChooser = new SendableChooser<>();
    for (TestableHardwareI subsystem : subsystemManager.getSubsystemList()) {
      testCommandChooser.addOption(subsystem.getName(), subsystem.getTestCommand());
    }
    return testCommandChooser;
  }

  public Command getWheelCharacterizationCommand() {
    WheelDiameterCharacterizer characterizer = new WheelDiameterCharacterizer();
    return new SequentialCommandGroup(
        getSubsystemTestMessageCommand("Preparing to move."),
        new InstantCommand(() -> {
          driveSubsystem.pathFollowDrive(new ChassisSpeeds(0.0, 0.0, 0.35));
        }, driveSubsystem),
        getSubsystemTestMessageCommand("Waiting for system movement to stabilize."),
        new WaitCommand(5),
        getSubsystemTestMessageCommand("Taking initial measurement."),
        new InstantCommand(() -> {
          characterizer.getInitialMeasurements();
        }),
        getSubsystemTestMessageCommand("Generating deltas."),
        new WaitCommand(120),
        getSubsystemTestMessageCommand("Calculating Results."),
        getSubsystemTestMessageCommand(() -> characterizer.updateAndCalculate()),
        new InstantCommand(() -> {
          driveSubsystem.pathFollowDrive(new ChassisSpeeds(0.0, 0.0, 0.0));
        }, driveSubsystem));
  }

  public Command getFullShootCommand() {
    return new VirtualTargetAutoShootCommand(subsystemManager.getShooterSubsystem(),
        subsystemManager.getHoodSubsystem(), subsystemManager.getTransferSubsystem(),
        subsystemManager.getTurretSubsystem());
  }

  public Command getRotateForBumpCommand() {
    return new RotateToAngleCommand(() -> {

      double angle = RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees);

      angle = angle % 360;

      if (angle >= 180.0) {
        angle -= 360.0;
      } else if (angle < -180.0) {
        angle += 360.0;
      }

      if (angle >= 0 && angle < 90) {
        return 45;
      } else if (angle >= 90 && angle < 180) {
        return 135;
      } else if (angle < 0 && angle >= -90) {
        return -45;
      } else {
        return -135;
      }
    });
  }

  private Pose3d getSnowblowTarget(Pose2d robotPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      if (Meters.of(robotPose.getY()).in(Inches) > RobotConstants.ODOMETRY.FIELD_WIDTH_INCHES / 2) {
        return RobotConstants.TURRET.RED_SNOWBLOW_TARGET_TOP;
      } else {
        return RobotConstants.TURRET.RED_SNOWBLOW_TARGET_BOTTOM;
      }
    } else {
      if (Meters.of(robotPose.getY()).in(Inches) > RobotConstants.ODOMETRY.FIELD_WIDTH_INCHES / 2) {
        return RobotConstants.TURRET.BLUE_SNOWBLOW_TARGET_TOP;
      } else {
        return RobotConstants.TURRET.BLUE_SNOWBLOW_TARGET_BOTTOM;
      }
    }
  }

  public Command getSnowblowCommand() {
    return new VirtualTargetAutoShootCommand(subsystemManager.getShooterSubsystem(),
        subsystemManager.getHoodSubsystem(), subsystemManager.getTransferSubsystem(),
        subsystemManager.getTurretSubsystem(), true, () -> getSnowblowTarget(RobotIO.getInstance().getOdometryPose()));
  }

  public Command getPresetShootCommand(ShotData preset) {
    if (preset == RobotConstants.SHOOTER.SHOT_PRESET_ONE) {
      return new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
          subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(),
          RobotIO.getInstance().getTurretOutput().getCurrentPosition(),
          RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetOneRPM")),
          Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetOneDegrees")),
          false);
    } else if (preset == RobotConstants.SHOOTER.SHOT_PRESET_TWO) {
      return new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
          subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(),
          RobotIO.getInstance().getTurretOutput().getCurrentPosition(),
          RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetTwoRPM")),
          Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetTwoDegrees")), false);
    } else {
      Supplier<TurretCalculator> turretCalculatorSupplier = () -> {
        Pose3d target = getSnowblowTarget(RobotIO.getInstance().getOdometryPose());

        Pose3d shooterCurrentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
            .transformBy(RobotConstants.SHOOTER.SHOT_TRANSFORM);
        return new TurretCalculator(target.toPose2d(),
            RobotIO.getInstance().getOdometryPose(), RobotIO.getInstance().getDriveOutput().getSpeeds());
      };

      Supplier<ShooterCalculator> shooterCalculatorSupplier = () -> {
        Pose2d target2d = RobotIO.getInstance().getOdometryPose();

        Pose3d target = new Pose3d(target2d.getX(), target2d.getY(), 0.0, new Rotation3d());

        Pose3d shooterCurrentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
            .transformBy(RobotConstants.SHOOTER.SHOT_TRANSFORM);

        return new ShooterCalculator(
            ChassisSpeeds.fromRobotRelativeSpeeds(RobotIO.getInstance().getDriveOutput().getSpeeds(),
                RobotIO.getInstance().getOdometryPose().getRotation()),
            shooterCurrentPose, target,
            Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS), RobotConstants.SHOOTER.MAX_RPM,
            RobotConstants.SHOOTER.MIN_RPM, RobotConstants.SHOOTER.MAX_SHOT_DISTANCE,
            RobotConstants.SHOOTER.MIN_SHOT_DISTANCE);
      };
      return new ShootAtTargetCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
          subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(), turretCalculatorSupplier,
          shooterCalculatorSupplier, true);
    }
  }

  public Command getStopShootingCommand() {
    return new ManualShootCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
        subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(), Degrees.of(0.0), RPM.of(0.0),
        Degree.of(0.0), false);
  }

  private Command getSubsystemTestMessageCommand(String message) {
    return new InstantCommand(() -> {
      Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, message);
    });
  }

  private Command getSubsystemTestMessageCommand(DoubleSupplier message) {
    return new InstantCommand(() -> {
      Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, "" + (message.getAsDouble() * 2));
    });
  }

  public Command getFountainCommand() {
    return new ParallelCommandGroup(
        new RunIntakeCommand(subsystemManager.getIntakeSubsystem()),
        new RunTransferCommand(subsystemManager.getTransferSubsystem()),
        new RunShooterAtLiveSpeedCommand(subsystemManager.getShooterSubsystem()),
        new ManualTurretCommand(subsystemManager.getTurretSubsystem(), 0),
        new ManualHoodCommand(subsystemManager.getHoodSubsystem(), 14.9));
  }
}
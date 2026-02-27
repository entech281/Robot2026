package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.entech.TestableHardwareI;
import frc.entech.commands.AutonomousException;
import frc.entech.commands.InstantAnytimeCommand;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.commands.GyroResetByAngleCommand;
import frc.robot.commands.HomeTurretCommand;
import frc.robot.commands.FaceTargetLocationTurretCommand;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.commands.RunShooterAtLiveSpeedCommand;
import frc.robot.commands.RunTestCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.livetuning.WheelDiameterCharacterizer;
import frc.robot.operation.UserPolicy;
import frc.robot.processors.OdometryProcessor;
import frc.robot.sensors.gyro.GyroSensor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ShooterCalculator;

import frc.robot.util.TurretCalculator;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;

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
        // .setDefaultCommand(new RunShooterAtLiveSpeedCommand(subsystemManager.getShooterSubsystem()));
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
    this.testChooser = getTestCommandChooser();
    testChooser.addOption("All tests", getTestCommand());
    Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, "No Current Test");
    SmartDashboard.putData("Test Chooser", testChooser);
    Shuffleboard.getTab("stuffs").add("Run Test", new RunTestCommand(testChooser));
    Shuffleboard.getTab("stuffs").add("Home Turret", new HomeTurretCommand(subsystemManager.getTurretSubsystem()));

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

    NamedCommands.registerCommand("example", Commands.deferredProxy(Commands::none));

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
        new WaitCommand(20),
        getSubsystemTestMessageCommand("Calculating Results."),
        getSubsystemTestMessageCommand(() -> characterizer.updateAndCalculate()),
        new InstantCommand(() -> {
          driveSubsystem.pathFollowDrive(new ChassisSpeeds(0.0, 0.0, 0.0));
        }, driveSubsystem));
  }

  public Command getFullShootCommand() {
    Pose3d target;

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      target = RobotConstants.TURRET.RED_HUB_LOCATION;
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      target = RobotConstants.TURRET.BLUE_HUB_LOCATION;
    } else {
      return Commands.none();
    }

    Pose3d shooterCurrentPose = new Pose3d(RobotIO.getInstance().getOdometryPose())
        .transformBy(RobotConstants.SHOOTER.SHOT_TRANSFORM);

    Supplier<ShooterCalculator> shooterCalculatorSupplier = () -> new ShooterCalculator(
        RobotIO.getInstance().getGyroOutput().getChassisSpeeds(), shooterCurrentPose, target);
    Supplier<TurretCalculator> turretCalculatorSupplier = () -> new TurretCalculator(target.toPose2d(),
        RobotIO.getInstance().getOdometryPose());

    return new ShootAtTargetCommand(subsystemManager.getShooterSubsystem(), subsystemManager.getHoodSubsystem(),
        subsystemManager.getTransferSubsystem(), subsystemManager.getTurretSubsystem(), turretCalculatorSupplier,
        shooterCalculatorSupplier);

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
}
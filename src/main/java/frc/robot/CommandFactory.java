package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.entech.commands.AutonomousException;
import frc.entech.commands.InstantAnytimeCommand;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.commands.GyroResetByAngleCommand;
import frc.robot.commands.RunTestCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.livetuning.WheelDiameterCharacterizer;
import frc.robot.operation.UserPolicy;
import frc.robot.processors.OdometryProcessor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.navx.NavXSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;

@SuppressWarnings("unused")
public class CommandFactory {
  private final DriveSubsystem driveSubsystem;
  private final NavXSubsystem navXSubsystem;
  private final OdometryProcessor odometry;
  private final SubsystemManager subsystemManager;
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> testChooser;

  public CommandFactory(SubsystemManager subsystemManager, OdometryProcessor odometry) {
    this.driveSubsystem = subsystemManager.getDriveSubsystem();
    this.navXSubsystem = subsystemManager.getNavXSubsystem();
    this.odometry = odometry;
    this.subsystemManager = subsystemManager;

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
    auto.addCommands(new GyroResetByAngleCommand(navXSubsystem, odometry, autoChooser.getSelected().getName()));
    auto.addCommands(new WaitCommand(0.5));
    auto.addCommands(autoChooser.getSelected());
    return auto;
  }

  public Command getTestCommand() {
    SequentialCommandGroup allTests = new SequentialCommandGroup();
    for (EntechSubsystem<?, ?> subsystem : subsystemManager.getSubsystemList()) {
      if (subsystem.isEnabled()) {
        addSubsystemTest(allTests, subsystem);
      }
    }
    allTests.addCommands(Commands.runOnce(() -> Logger
        .recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST, "No Current Tests.")));
    return allTests;
  }

  private static void addSubsystemTest(SequentialCommandGroup group,
      EntechSubsystem<?, ?> subsystem) {

    group.addCommands(
        Commands.runOnce(() -> Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
            String.format("%s: Start", subsystem.getName()))),
        subsystem.getTestCommand(),
        Commands.runOnce(() -> Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
            String.format("%s: Finished", subsystem.getName()))));
  }

  private SendableChooser<Command> getTestCommandChooser() {
    SendableChooser<Command> testCommandChooser = new SendableChooser<>();
    for (EntechSubsystem<?, ?> subsystem : subsystemManager.getSubsystemList()) {
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
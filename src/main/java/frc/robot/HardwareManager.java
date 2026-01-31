// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.entech.TestableHardwareI;
import frc.robot.io.RobotIO;
import frc.robot.sensors.navx.NavXSensor;
import frc.robot.sensors.vision.VisionSensor;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Manages the subsystems and the interactions between them.
 */
public class HardwareManager {
  private final VisionSensor visionSensor = new VisionSensor();
  private final NavXSensor navXSensor = new NavXSensor();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final TransferSubsystem transferSubsystem= new TransferSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

  public HardwareManager() {
    navXSensor.initialize();
    driveSubsystem.initialize();
    shooterSubsystem.initialize();
    visionSensor.initialize();
    hoodSubsystem.initialize();
    turretSubsystem.initialize();
    hopperSubsystem.initialize();
    intakeSubsystem.initialize();
    climbSubsystem.initialize();
    transferSubsystem.initialize();

    periodic();
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public NavXSensor getNavXSubsystem() {
    return navXSensor;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public VisionSensor getVisionSensor() {
    return visionSensor;
  }

  public HoodSubsystem getHoodSubsystem() {
    return hoodSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public HopperSubsystem getHopperSubsystem() {
    return hopperSubsystem;
  }

  public ClimbSubsystem getClimbSubsystem() {
    return climbSubsystem;
  }

  public TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public TransferSubsystem getTransferSubsystem() {
    return transferSubsystem;
  }

  public List<TestableHardwareI> getSubsystemList() {
    ArrayList<TestableHardwareI> r = new ArrayList<>();
    r.add(driveSubsystem);
    r.add(shooterSubsystem);
    r.add(intakeSubsystem);
    r.add(transferSubsystem);
    r.add(turretSubsystem);
    r.add(hoodSubsystem);
    r.add(hopperSubsystem);
    r.add(climbSubsystem);
    r.add(navXSensor);
    r.add(visionSensor);

    return r;
  }

  public final void periodic() {
    RobotIO outputs = RobotIO.getInstance();

    outputs.updateDrive(driveSubsystem.getOutputs());

    outputs.updateNavx(navXSensor.getOutputs());

    outputs.updateShooter(shooterSubsystem.getOutputs());

    outputs.updateVision(visionSensor.getOutputs());

    outputs.updateClimb(climbSubsystem.getOutputs());

    outputs.updateHopper(hopperSubsystem.getOutputs());

    outputs.updateHood(hoodSubsystem.getOutputs());

    outputs.updateIntake(intakeSubsystem.getOutputs());

    outputs.updateTransfer(transferSubsystem.getOutputs());

    outputs.updateTurret(turretSubsystem.getOutputs());
  }
}

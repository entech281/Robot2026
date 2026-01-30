// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SubsystemInput;
import frc.entech.subsystems.SubsystemOutput;
import frc.robot.io.RobotIO;
import frc.robot.sensors.navx.NavXSensor;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Manages the subsystems and the interactions between them.
 */
public class HardwareManager {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final NavXSensor navXSensor = new NavXSensor();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public HardwareManager() {
    navXSensor.initialize();
    driveSubsystem.initialize();
    shooterSubsystem.initialize();

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

  public List<EntechSubsystem<? extends SubsystemInput, ? extends SubsystemOutput>> getSubsystemList() {
    ArrayList<EntechSubsystem<? extends SubsystemInput, ? extends SubsystemOutput>> r = new ArrayList<>();
    r.add(driveSubsystem);
    r.add(shooterSubsystem);

    return r;
  }

  public final void periodic() {
    RobotIO outputs = RobotIO.getInstance();

    outputs.updateDrive(driveSubsystem.getOutputs());

    outputs.updateNavx(navXSensor.getOutputs());

    outputs.updateShooter(shooterSubsystem.getOutputs());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.entech.TestableHardwareI;
import frc.robot.io.RobotIO;
import frc.robot.sensors.gyro.GyroSensor;
import frc.robot.sensors.vision.VisionSensor;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Manages the subsystems and the interactions between them.
 */
public class HardwareManager {
  private final VisionSensor visionSensor = new VisionSensor();
  private final GyroSensor gyroSensor = new GyroSensor();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final PrototypeSubsystem prototypeSubsystem = new PrototypeSubsystem();
  private final PrototypeSubsystem2 prototypeSubsystem2 = new PrototypeSubsystem2();


  public HardwareManager() {
    gyroSensor.initialize();
    driveSubsystem.initialize();
    visionSensor.initialize();
    prototypeSubsystem.initialize();
    prototypeSubsystem2.initialize();
    periodic();
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
  
  public PrototypeSubsystem getPrototypeSubsystem() {
    return prototypeSubsystem;
  }

  public PrototypeSubsystem2 getPrototypeSubsystem2() {
    return prototypeSubsystem2;
  }

  public GyroSensor getGyroSubsystem() {
    return gyroSensor;
  }


  public VisionSensor getVisionSensor() {
    return visionSensor;
  }



  public List<TestableHardwareI> getSubsystemList() {
    ArrayList<TestableHardwareI> r = new ArrayList<>();
    r.add(driveSubsystem);
    r.add(prototypeSubsystem);
    r.add(prototypeSubsystem2);
    r.add(gyroSensor);
    r.add(visionSensor);
    return r;
  }

  public final void periodic() {
    RobotIO outputs = RobotIO.getInstance();

    outputs.updateDrive(driveSubsystem.getOutputs());
    outputs.updatePrototype(prototypeSubsystem.getOutputs());
    outputs.updatePrototype2(prototypeSubsystem2.getOutputs());

    outputs.updateGyro(gyroSensor.getOutputs());


    outputs.updateVision(visionSensor.getOutputs());
  }
}

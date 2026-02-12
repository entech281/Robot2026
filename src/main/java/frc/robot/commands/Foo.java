package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hood.HoodOutput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShooterCalculator;

public class Foo extends EntechCommand {

    private double shooterRPM;
    private final ShooterSubsystem shooterSubsystem;
    private Pose3d target;
    private final TurretSubsystem turretSubsystem;

    public Foo(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, Pose3d target) {
        super();
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.target = target;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {}

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
          if (!new ShooterCalculator().hasValidShot()) return;
        new ParallelCommandGroup(
      

        new ParallelCommandGroup(
            new ShooterRPM(shooterSubsystem, this.shooterRPM),
            new FaceTargetLocationTurretCommand(turretSubsystem, this.target),
            new HoodAtTarget();
        );

        new WaitUntilCommand(turretOutput.isFacingTarget && shooterOutput.isAtSpeed);

        new RunTransferCommand();
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return !new ShooterCalculator().hasValidShot() || buttonReleased;
    }
  

    kickerss
    shooterSS
    turretSubsystem
    hoodSS

    shooterOutput
    turretOutput
    HoodOutput
    shoooterout

    Chassis
    targetPose (needs alliance)
    currentPose
    
    button

    
}

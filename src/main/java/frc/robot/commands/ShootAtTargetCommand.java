package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootAtTargetCommand extends EntechCommand {

    private final HoodSubsystem hoodSS;
    private final Pose3d targetPose;
    private final ShooterSubsystem shooterSS;

    public ShootAtTargetCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, Pose3d target) {
        super(hoodSubsystem);
        this.hoodSS = hoodSubsystem;
        this.targetPose = target;
        this.shooterSS = shooterSubsystem;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
    
    
}

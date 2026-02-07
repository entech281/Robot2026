package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShotData;

public class ShootAtTargetCommand extends EntechCommand {

    private final HoodSubsystem hoodSS;
    private final Pose3d targetPose;
    private final ShooterSubsystem shooterSS;
    private ShooterCalculator calculator = new ShooterCalculator();
    private ShotData targetShot;

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
        Pose3d currentPose = new Pose3d(RobotIO.getInstance().getOdometryPose()).transformBy(RobotConstants.SHOOTER.SHOT_TRANSFORM);
        this.targetShot = calculator.calculateShot(RobotIO.getInstance().getNavXOutput().getChassisSpeeds(), currentPose, targetPose);
    }

    @Override
    public boolean isFinished() {
        return RobotIO.getInstance().getHoodOutput().isAtTargetAngle() &&
               RobotIO.getInstance().getShooterOutput().isAtSpeed();
    }
    
    
}

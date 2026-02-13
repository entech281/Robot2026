package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
    private final ShooterSubsystem shooterSS;
    private ShooterCalculator calculator = new ShooterCalculator();
    private ShooterCalculator shooterCalculator;

    public ShootAtTargetCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, ShooterCalculator calculator) {
        super(hoodSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.shooterCalculator = calculator;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        calculator.calculateShot();
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
}

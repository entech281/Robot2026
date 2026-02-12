package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.TurretCalculator;

public class FaceTargetLocationTurretCommand extends EntechCommand {

    private final TurretSubsystem turretSubsystem;
    private final TurretInput turretInput = new TurretInput();
    private final Pose2d target;
    private TurretCalculator calculator = new TurretCalculator();


    public FaceTargetLocationTurretCommand(TurretSubsystem turretSubsystem, Pose2d target) {
        super(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d robotPose = RobotIO.getInstance().getOdometryPose();
        
        double targetAngle = calculator.calculateTargetTurretAngle(target, robotPose);
        
        turretInput.setRequestedPosition(targetAngle);
        turretSubsystem.updateInputs(turretInput);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        //never ends, continously tracks target
        return turretSubsystem.getOutputs().isAtRequestedPosition();
    }
}

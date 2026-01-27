package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class FaceTargetLocationTurretCommand extends EntechCommand {

    private final TurretSubsystem turretSubsystem;
    private final TurretInput turretInput = new TurretInput();
    private final Pose2d target;


    public FaceTargetLocationTurretCommand(TurretSubsystem turretSubsystem, Pose2d target) {
        super(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //TODO: Find out what happens if I request and angle that is outside
        //of the range of motion
        Pose2d robotPose = RobotIO.getInstance().getOdometryPose();
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        //TODO: Minus or plus I'm not really sure
        turretInput.setRequestedPosition(angleToTarget + robotPose.getRotation().getDegrees());
        turretSubsystem.updateInputs(turretInput);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        //never ends, continously tracks target
        return false;
    }
}

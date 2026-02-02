package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterAtLiveSpeedCommand extends EntechCommand {
    private final ShooterSubsystem shooterSubsystem;

    public RunShooterAtLiveSpeedCommand(ShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        ShooterInput si = new ShooterInput();
        si.setSpeed(0.0);
        shooterSubsystem.updateInputs(si);
    }

    @Override
    public void execute() {
        ShooterInput si = new ShooterInput();
        si.setSpeed(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        shooterSubsystem.updateInputs(si);
    }

    @Override
    public void initialize() {
        ShooterInput si = new ShooterInput();
        si.setSpeed(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        shooterSubsystem.updateInputs(si);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

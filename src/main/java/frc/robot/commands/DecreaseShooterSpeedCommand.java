package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Decreases the shooter speed by a fixed decrement.
 * Useful for tuning: press button repeatedly to find optimal RPM.
 * 
 * Speed is read from LiveTuningHandler and decremented each press.
 * To reset speed between test runs, adjust via dashboard or tuning constants.
 */
public class DecreaseShooterSpeedCommand extends EntechCommand {
    private final ShooterSubsystem shooter;
    private static final String SPEED_KEY = "ShooterSubsystem/SetSpeed";
    private static final double INCREMENT = 500.0; // RPM decrement per press

    public DecreaseShooterSpeedCommand(ShooterSubsystem shooter) {
        super(shooter);
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        double currentSpeed = LiveTuningHandler.getInstance().getValue(SPEED_KEY);
        double newSpeed = Math.max(0, currentSpeed - INCREMENT); // Don't go below 0
        
        // Run shooter at new speed
        ShooterInput input = new ShooterInput();
        input.setSpeed(newSpeed);
        shooter.updateInputs(input);
        
        // Note: The new speed will be used for subsequent runs
        // Next time this command initializes, it will read the new speed from LiveTuningHandler
    }

    @Override
    public void execute() {
        // Keep running at current speed
        double currentSpeed = LiveTuningHandler.getInstance().getValue(SPEED_KEY);
        double newSpeed = Math.max(0, currentSpeed - INCREMENT);
        ShooterInput input = new ShooterInput();
        input.setSpeed(newSpeed);
        shooter.updateInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        // Keep shooter running at new speed - don't stop it
    }

    @Override
    public boolean isFinished() {
        return true; // Finish immediately after initialize
    }
}
package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Runs the shooter at a configured speed in the specified direction.
 * Useful for tuning: press button to spin shooter.
 */
public class RunShooterCommand extends EntechCommand {
    private final ShooterSubsystem shooter;
    private boolean direction;
    private static final String KEY_STRING = "ShooterSubsystem/SetSpeed";

    public RunShooterCommand(ShooterSubsystem shooter) {
        this(shooter, true);
    }

    /**
     * @param shooter the shooter subsystem
     * @param direction true for forward spin, false for reverse
     */
    public RunShooterCommand(ShooterSubsystem shooter, boolean direction) {
        super(shooter);
        this.shooter = shooter;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        ShooterInput input = new ShooterInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (direction) {
            input.setSpeed(speed);
        } else {
            input.setSpeed(-speed);
        }
        shooter.updateInputs(input);
    }

    @Override
    public void execute() {
        ShooterInput input = new ShooterInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (direction) {
            input.setSpeed(speed);
        } else {
            input.setSpeed(-speed);
        }
        shooter.updateInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.updateInputs(new ShooterInput());
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}
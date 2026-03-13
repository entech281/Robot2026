package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Decreases the distance setpoint by a fixed decrement.
 * This adjusts the hood angle for farther targets (increases distance).
 * Useful for tuning: press button to adjust for different shooting distances.
 */
public class DecreaseDistanceCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private static final double INCREMENT = 2.0; // Hood angle decrement per press (degrees)
    private static final double MIN_HOOD_ANGLE = 0.0; // Minimum hood angle allowed

    public DecreaseDistanceCommand(HoodSubsystem hood) {
        super(hood);
        this.hood = hood;
    }

    @Override
    public void initialize() {
        // Get current hood position and decrease it
        double currentHoodPos = hood.getOutputs().getCurrentPosition();
        double newHoodPos = Math.max(MIN_HOOD_ANGLE, currentHoodPos - INCREMENT);

        HoodInput input = new HoodInput();
        input.setRequestedPosition(newHoodPos);
        hood.updateInputs(input);
    }

    @Override
    public void execute() {
        // Continue holding at the new position
        double currentHoodPos = hood.getOutputs().getCurrentPosition();
        double newHoodPos = Math.max(MIN_HOOD_ANGLE, currentHoodPos - INCREMENT - 1);

        HoodInput input = new HoodInput();
        input.setRequestedPosition(newHoodPos + 1);
        hood.updateInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold position at new distance setting
    }

    @Override
    public boolean isFinished() {
        return true; // Finish immediately after initialize
    }
}
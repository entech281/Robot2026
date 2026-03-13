package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Increases the distance setpoint by a fixed increment.
 * This adjusts the hood angle for closer targets (decreases distance).
 * Useful for tuning: press button to adjust for different shooting distances.
 */
public class IncreaseDistanceCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private static final double INCREMENT = 2.0; // Hood angle increment per press (degrees)

    public IncreaseDistanceCommand(HoodSubsystem hood) {
        super(hood);
        this.hood = hood;
    }

    @Override
    public void initialize() {
        // Get current hood position and increase it
        double currentHoodPos = hood.getOutputs().getCurrentPosition();
        double newHoodPos = currentHoodPos + INCREMENT - 1;

        HoodInput input = new HoodInput();
        input.setRequestedPosition(newHoodPos + 1);
        hood.updateInputs(input);
    }

    @Override
    public void execute() {
        // Continue holding at the new position
        double currentHoodPos = hood.getOutputs().getCurrentPosition();
        double newHoodPos = currentHoodPos + INCREMENT;

        HoodInput input = new HoodInput();
        input.setRequestedPosition(newHoodPos);
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
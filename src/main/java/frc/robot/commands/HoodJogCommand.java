package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Jog the hood by a small relative step while the command is active.
 * Hold the button to nudge repeatedly.
 */
public class HoodJogCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private final double stepDegrees;
    private double targetPosition = 0.0;

    /**
     * @param hood the hood subsystem
     * @param stepDegrees positive to move up, negative to move down
     */
    public HoodJogCommand(HoodSubsystem hood, double stepDegrees) {
        super(hood);
        this.hood = hood;
        this.stepDegrees = stepDegrees;
    }

    @Override
    public void initialize() {
        // Start from current position
        targetPosition = hood.getOutputs().getCurrentPosition();
    }

    @Override
    public void execute() {
        // Increment target position by step
        targetPosition += stepDegrees;
        
        // Apply the target position
        HoodInput in = new HoodInput();
        in.setRequestedPosition(targetPosition);
        hood.updateInputs(in);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold at current target - don't reset to zero
        HoodInput in = new HoodInput();
        in.setRequestedPosition(targetPosition);
        hood.updateInputs(in);
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}
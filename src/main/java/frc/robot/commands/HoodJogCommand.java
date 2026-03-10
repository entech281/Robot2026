package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Jog the hood by a small relative step while the command is active.
 * Hold the button to nudge repeatedly.
 * Step size is defined in RobotConstants.HOOD.HOOD_JOG_STEP_DEGREES
 */
public class HoodJogCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private final int direction; // -1 or 1 to control direction
    private double targetPosition = 0.0;
    private static final double STEP_DEGREES = RobotConstants.HOOD.HOOD_JOG_STEP_DEGREES;

    /**
     * @param hood the hood subsystem
     * @param direction -1 to move down, 1 to move up
     */
    public HoodJogCommand(HoodSubsystem hood, int direction) {
        super(hood);
        this.hood = hood;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        // Start from current position
        targetPosition = hood.getOutputs().getCurrentPosition();
    }

    @Override
    public void execute() {
        // Increment target position by step in the specified direction
        targetPosition += (direction * STEP_DEGREES);
        
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
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperSubsystem;

/**
 * Drops the hopper until the lower limit is reached, then raises it for a short duration
 * and stops. Useful for a single-cycle test.
 */
public class DropThenRaiseHopper extends EntechCommand {
    private final HopperSubsystem hopper;
    private final Timer timer = new Timer();
    private enum State { DROPPING, RAISING, DONE }
    private State state = State.DROPPING;
    private final double raiseDurationSeconds;

    public DropThenRaiseHopper(HopperSubsystem hopper) {
        this(hopper, 0.5);
    }

    public DropThenRaiseHopper(HopperSubsystem hopper, double raiseDurationSeconds) {
        super(hopper);
        this.hopper = hopper;
        this.raiseDurationSeconds = raiseDurationSeconds;
    }

    @Override
    public void initialize() {
        state = State.DROPPING;
        timer.reset();
        timer.stop();
        
        // Start dropping
        HopperInput in = new HopperInput();
        in.setSpeed(1.0);
        hopper.updateInputs(in);
    }

    @Override
    public void execute() {
        if (state == State.DROPPING) {
            // Check if we've reached the lower limit via hopper outputs
            boolean atLower = hopper.getOutputs().isAtLowerLimit();
            if (atLower) {
                // Transition to raising
                state = State.RAISING;
                timer.reset();
                timer.start();
                HopperInput in = new HopperInput();
                in.setSpeed(-1.0);
                hopper.updateInputs(in);
            } else {
                // Keep dropping
                HopperInput in = new HopperInput();
                in.setSpeed(1.0);
                hopper.updateInputs(in);
            }
        } else if (state == State.RAISING) {
            if (timer.hasElapsed(raiseDurationSeconds)) {
                // Done with cycle
                state = State.DONE;
                hopper.updateInputs(new HopperInput());
            } else {
                // Continue raising
                HopperInput in = new HopperInput();
                in.setSpeed(-1.0);
                hopper.updateInputs(in);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        hopper.updateInputs(new HopperInput());
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }
}

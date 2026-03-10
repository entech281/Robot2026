package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperSubsystem;

/**
 * Simple hopper toggle: if not at lower limit, run down, otherwise run up.
 * Uses the real `HopperOutput` reported via `RobotIO` rather than a new object.
 */
public class DropHopper extends EntechCommand {
    private final HopperSubsystem hopper;

    public DropHopper(HopperSubsystem hopper) {
        super(hopper);
        this.hopper = hopper;
    }

    @Override
    public void execute() {
        HopperInput input = new HopperInput();
        var output = RobotIO.getInstance().getHopperOutput();

        boolean atLower = output != null && output.isAtLowerLimit();

        if (!atLower) {
            input.setSpeed(1);
        } else {
            input.setSpeed(-1);
        }

        hopper.updateInputs(input);
    }

    @Override
    public void initialize() {
        // same as execute for a momentary press
        execute();
    }

    @Override
    public void end(boolean interrupted) {
        hopper.updateInputs(new HopperInput());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


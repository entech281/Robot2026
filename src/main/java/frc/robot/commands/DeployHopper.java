package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class DeployHopper extends EntechCommand {
    private static final double DEPLOY_SPEED = -0.3; // Negative = downward, tune as needed

    private final HopperSubsystem hopperSubsystem;

    public DeployHopper(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        HopperInput input = new HopperInput();
        input.setSpeed(DEPLOY_SPEED);
        hopperSubsystem.updateInputs(input);
    }

    @Override
    public void execute() {
        // periodic() in the subsystem handles stopping when limit is hit,
        // but we also check here so isFinished() can end the command cleanly
    }

    @Override
    public boolean isFinished() {
        return hopperSubsystem.getOutputs().isAtLowerLimit();
    }

    @Override
    public void end(boolean interrupted) {
        HopperInput input = new HopperInput();
        input.setSpeed(0.0);
        hopperSubsystem.updateInputs(input);
    }
}

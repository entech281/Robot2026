package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class DeployHopper extends EntechCommand {
    boolean direction;

    private final HopperSubsystem hopperSubsystem;

    public DeployHopper(HopperSubsystem hopperSubsystem) {
        this(hopperSubsystem, true);
    }

    /**
     * 
     * @param hopperSubsystem
     * @param direction true for deploy, false for collapse
     */
    public DeployHopper(HopperSubsystem hopperSubsystem, boolean direction) {
        super(hopperSubsystem);
        this.hopperSubsystem = hopperSubsystem;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        HopperInput input = new HopperInput();

        if (direction) {
            input.setSpeed(RobotConstants.HOPPER.DEPLOY_SPEED);
        } else {
            input.setSpeed(-RobotConstants.HOPPER.DEPLOY_SPEED);
        }
        hopperSubsystem.updateInputs(input);
    }

    @Override
    public void execute() {
        // periodic() in the subsystem handles stopping when limit is hit,
        // but we also check here so isFinished() can end the command cleanly
    }

    @Override
    public boolean isFinished() {
        return hopperSubsystem.getOutputs().isAtLowerLimit() || hopperSubsystem.getOutputs().isStalled();
    }

    @Override
    public void end(boolean interrupted) {
        HopperInput input = new HopperInput();
        input.setSpeed(0.0);
        hopperSubsystem.updateInputs(input);
    }
}

package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Jog the hood by a small relative step while the command is active.
 * Hold the button to nudge repeatedly.
 */
public class HoodJogCommand extends EntechCommand {
	private final HoodSubsystem hood;
	private final double stepDegrees;

	public HoodJogCommand(HoodSubsystem hood, double stepDegrees) {
		super(hood);
		this.hood = hood;
		this.stepDegrees = stepDegrees;
	}

	@Override
	public void initialize() {
		// no-op
	}

	@Override
	public void execute() {
		HoodInput in = new HoodInput();
		double current = RobotIO.getInstance().getHoodOutput().getCurrentPosition();
		in.setRequestedPosition(current + stepDegrees);
		hood.updateInputs(in);
	}

	@Override
	public void end(boolean interrupted) {
		hood.updateInputs(new HoodInput());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}


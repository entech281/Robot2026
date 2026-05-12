package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

/**
 * Jog the hood by a small relative step while the command is active.
 * Hold the button to nudge repeatedly.
 * Step size is defined in RobotConstants.HOOD.HOOD_JOG_STEP_DEGREES
 */
public class HoodJogCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private final double stepDegrees;
    private double current;

    /**
     * @param hood        the hood subsystem
     * @param stepDegrees positive to move up, negative to move down
     */
    public HoodJogCommand(HoodSubsystem hood, double stepDegrees) {
        super(hood);
        this.hood = hood;
        this.stepDegrees = stepDegrees;
    }

    @Override
    public void initialize() {
        current = RobotIO.getInstance().getHoodOutput().getCurrentPosition();
        HoodInput in = new HoodInput();
        in.setRequestedPosition(current + stepDegrees);
        hood.acceptInputs(in);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

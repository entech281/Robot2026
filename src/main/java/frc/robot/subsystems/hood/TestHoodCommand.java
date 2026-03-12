package frc.robot.subsystems.hood;

import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.RobotConstants;

public class TestHoodCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private final StoppingCounter counter = new StoppingCounter(RobotConstants.TEST_CONSTANTS.STANDARD_TEST_LENGTH);

    public TestHoodCommand(HoodSubsystem hood) {
        super(hood);
        this.hood = hood;
    }

    @Override
    public void initialize() {
        counter.reset();
        HoodInput in = new HoodInput();
        in.setRequestedPosition(10.0);
        hood.updateInputs(in);
    }

    @Override
    public void end(boolean interrupted) {
        HoodInput in = new HoodInput();
        in.setRequestedPosition(0.0);
        hood.updateInputs(in);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
}

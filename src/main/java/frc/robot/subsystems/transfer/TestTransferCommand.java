package frc.robot.subsystems.transfer;

import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.RobotConstants;

public class TestTransferCommand extends EntechCommand {
    private final TransferSubsystem transferSubsystem;
    private final StoppingCounter counter = new StoppingCounter(RobotConstants.TEST_CONSTANTS.STANDARD_TEST_LENGTH);

    public TestTransferCommand(TransferSubsystem transferSubsystem) {
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize() {
        counter.reset();
        TransferInput input = new TransferInput();
        input.setSpeed(0.5);
        transferSubsystem.acceptInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        TransferInput input = new TransferInput();
        input.setSpeed(0.0);
        transferSubsystem.acceptInputs(input);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
}

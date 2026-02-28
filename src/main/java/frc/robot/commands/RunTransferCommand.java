package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class RunTransferCommand extends EntechCommand {
    private final TransferSubsystem transfer;
    private boolean direction;

    public RunTransferCommand(TransferSubsystem transfer) {
        this(transfer, true);
    }

    /**
     * 
     * @param transfer
     * @param direction true for intake false for extake
     */
    public RunTransferCommand(TransferSubsystem transfer, boolean direction) {
        super(transfer);
        this.transfer = transfer;
        this.direction = direction;
    }

    @Override
    public void end(boolean interrupted) {
        transfer.updateInputs(new TransferInput());
    }

    @Override
    public void execute() {
        TransferInput input = new TransferInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
        }
        transfer.updateInputs(input);
    }

    public void runIntake(){
        TransferInput input = new TransferInput();
        input.setSpeed(0.5);
    }

    @Override
    public void initialize() {
        TransferInput input = new TransferInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
        }
        transfer.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class RunTransferCommand extends EntechCommand {
    private final TransferSubsystem transfer;

    public RunTransferCommand(TransferSubsystem transfer) {
        super(transfer);
        this.transfer = transfer;
    }

    @Override
    public void end(boolean interrupted) {
        TransferInput in = new TransferInput();
        in.setSpeed(0.0);
        transfer.updateInputs(in);
    }

    @Override
    public void execute() {
        TransferInput in = new TransferInput();
        in.setSpeed(-1.0);
        transfer.updateInputs(in);
    }

    @Override
    public void initialize() {
        TransferInput in = new TransferInput();
        in.setSpeed(-1.0);
        transfer.updateInputs(in);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

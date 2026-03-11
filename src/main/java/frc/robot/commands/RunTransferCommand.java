package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * Runs the transfer/conveyor at a configured speed in the specified direction.
 * Useful for tuning: press button to run transfer.
 */
public class RunTransferCommand extends EntechCommand {
    private final TransferSubsystem transfer;
    private boolean direction;
    private static final String KEY_STRING = "TransferSubsystem/SetSpeed";

    public RunTransferCommand(TransferSubsystem transfer) {
        this(transfer, true);
    }

    /**
     * @param transfer  the transfer subsystem
     * @param direction true for forward, false for reverse
     */
    public RunTransferCommand(TransferSubsystem transfer, boolean direction) {
        super(transfer);
        this.transfer = transfer;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        TransferInput input = new TransferInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (direction) {
            input.setSpeed(speed);
        } else {
            input.setSpeed(-speed);
        }
        transfer.updateInputs(input);
    }

    @Override
    public void execute() {
        TransferInput input = new TransferInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (!direction) {
            input.setSpeed(-speed);
        } else {
            input.setSpeed(speed);
        }
        transfer.updateInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        transfer.updateInputs(new TransferInput());
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}

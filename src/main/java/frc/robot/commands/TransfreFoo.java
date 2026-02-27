package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class TransfreFoo extends EntechCommand {
    private final TransferSubsystem transferSS;

    public TransfreFoo(TransferSubsystem transferSS) {
        super(transferSS);
        this.transferSS = transferSS;
    }

    @Override
    public void end(boolean interrupted) {
        TransferInput si = new TransferInput();
        si.setSpeed(0.0);
        transferSS.updateInputs(si);
    }

    @Override
    public void execute() {
        TransferInput si = new TransferInput();
        si.setSpeed(-1.0);
        transferSS.updateInputs(si);
    }

    @Override
    public void initialize() {
        TransferInput si = new TransferInput();
        si.setSpeed(-1.0);
        transferSS.updateInputs(si);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

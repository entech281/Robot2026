package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AimTurretLiveCommand extends EntechCommand {
    private final TurretSubsystem turret;

    public AimTurretLiveCommand(TurretSubsystem turret) {
        super(turret);
        this.turret = turret;
    }

    @Override
    public void end(boolean interrupted) {
        TurretInput in = new TurretInput();
        in.setActivate(false);
        in.setRequestedPosition(0.0);
        turret.updateInputs(in);
    }

    @Override
    public void execute() {
        TurretInput in = new TurretInput();
        in.setActivate(false);
        in.setRequestedPosition(LiveTuningHandler.getInstance().getValue("TurretSubsystem/LiveAngle"));
        turret.updateInputs(in);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}

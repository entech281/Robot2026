package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import static edu.wpi.first.units.Units.Degrees;

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
        in.setRequestedPosition(Degrees.of(0.0));
        turret.acceptInputs(in);
    }

    @Override
    public void execute() {
        TurretInput in = new TurretInput();
        in.setActivate(false);
        in.setRequestedPosition(Degrees.of(LiveTuningHandler.getInstance().getValue("TurretSubsystem/LiveAngle")));
        turret.acceptInputs(in);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}

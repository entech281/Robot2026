package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretContinuousNudgeCommand extends EntechCommand {
    private final TurretSubsystem turret;
    private final boolean direction;

    public TurretContinuousNudgeCommand(TurretSubsystem turret, boolean direction) {
        super(turret);
        this.turret = turret;
        this.direction = direction;
    }

    @Override
    public void execute() {
        TurretInput in = new TurretInput();
        if (direction) {
            in.setRequestedPosition(RobotIO.getInstance().getTurretOutput().getCurrentPosition()
                    .plus(Degrees.of(LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount"))));
        } else {
            in.setRequestedPosition(RobotIO.getInstance().getTurretOutput().getCurrentPosition()
                    .minus(Degrees.of(LiveTuningHandler.getInstance().getValue("TurretSubsystem/NudgeAmount"))));
        }
        turret.acceptInputs(in);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;

public class HoodContinuousNudgeCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private final boolean direction;

    public HoodContinuousNudgeCommand(HoodSubsystem hood, boolean direction) {
        super(hood);
        this.hood = hood;
        this.direction = direction;
    }

    @Override
    public void execute() {
        HoodInput in = new HoodInput();
        if (direction) {
            in.setRequestedPosition(RobotIO.getInstance().getHoodOutput().getCurrentPosition()
                    + LiveTuningHandler.getInstance().getValue("HoodSubsystem/NudgeAmount"));
        } else {
            in.setRequestedPosition(RobotIO.getInstance().getHoodOutput().getCurrentPosition()
                    - LiveTuningHandler.getInstance().getValue("HoodSubsystem/NudgeAmount"));
        }
        hood.acceptInputs(in);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

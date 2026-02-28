package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunHoodCommand extends EntechCommand {
    private final HoodSubsystem hood;
    private boolean direction;

    public RunHoodCommand(HoodSubsystem hood) {
        this(hood, true);
    }

    /**
     * 
     * @param intake
     * @param direction true for intake false for extake
     */
    public RunHoodCommand(HoodSubsystem hood, boolean direction) {
        super(hood);
        this.hood = hood;
        this.direction = direction;
    }

    @Override
    public void end(boolean interrupted) {
        hood.updateInputs(new HoodInput());
    }

    @Override
    public void execute() {
        HoodInput input = new HoodInput();
        if (direction) {
            input.setRequestedPosition(LiveTuningHandler.getInstance().getValue("HoodSubsystem/SetRequestedPosition"));
        } else {
            input.setRequestedPosition(-LiveTuningHandler.getInstance().getValue("HoodSubsystem/SetRequestedPosition"));
        }
        hood.updateInputs(input);
    }

    public void runHood(){
        HoodInput input = new HoodInput();
        input.setRequestedPosition(input.getRequestedPosition() + 5);
    }

    @Override
    public void initialize() {
        HoodInput input = new HoodInput();
        if (direction) {
            input.setRequestedPosition(LiveTuningHandler.getInstance().getValue("HoodSubsystem/SetRequestedPosition"));
        } else {
            input.setRequestedPosition(-LiveTuningHandler.getInstance().getValue("HoodSubsystem/SetRequestedPosition"));
        }
        hood.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

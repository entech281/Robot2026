package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends EntechCommand {
    private final IntakeSubsystem intakeSS;

    public RunIntakeCommand(IntakeSubsystem intake) {
        super(intake);
        intakeSS = intake;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.updateInputs(new IntakeInput());
    }

    @Override
    public void execute() {
        IntakeInput input = new IntakeInput();
        input.setSpeed(LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        intakeSS.updateInputs(input);
    }

    @Override
    public void initialize() {
        IntakeInput input = new IntakeInput();
        input.setSpeed(LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        intakeSS.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

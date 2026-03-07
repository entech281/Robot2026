package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends EntechCommand {
    private final IntakeSubsystem intakeSS;
    private boolean direction;

    public RunIntakeCommand(IntakeSubsystem intake) {
        this(intake, true);
    }

    /**
     * 
     * @param intake
     * @param direction true for intake false for extake
     */
    public RunIntakeCommand(IntakeSubsystem intake, boolean direction) {
        super(intake);
        intakeSS = intake;
        this.direction = direction;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.updateInputs(new IntakeInput());
    }

    @Override
    public void execute() {
        IntakeInput input = new IntakeInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        }
        intakeSS.updateInputs(input);
    }

    public void runIntake(){
        IntakeInput input = new IntakeInput();
        input.setSpeed(0.5);
    }

    @Override
    public void initialize() {
        IntakeInput input = new IntakeInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("IntakeSubsystem/SetSpeed"));
        }
        intakeSS.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

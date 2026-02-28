package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterCommand extends EntechCommand {
    private final ShooterSubsystem shooter;
    private boolean direction;

    public RunShooterCommand(ShooterSubsystem shooter) {
        this(shooter, true);
    }

    /**
     * 
     * @param intake
     * @param direction true for intake false for extake
     */
    public RunShooterCommand(ShooterSubsystem shooter, boolean direction) {
        super(shooter);
        this.shooter = shooter;
        this.direction = direction;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.updateInputs(new ShooterInput());
    }

    @Override
    public void execute() {
        ShooterInput input = new ShooterInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        }
        shooter.updateInputs(input);
    }

    public void runIntake(){
        ShooterInput input = new ShooterInput();
        input.setSpeed(0.5);
    }

    @Override
    public void initialize() {
        ShooterInput input = new ShooterInput();
        if (direction) {
            input.setSpeed(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        } else {
            input.setSpeed(-LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
        }
        shooter.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

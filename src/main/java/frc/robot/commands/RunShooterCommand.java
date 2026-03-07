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
        
        double postiveSpeed = LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed");
        double negativeSpeed = -LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed");  
        if (direction) {
            input.setSpeed(postiveSpeed);
        } else {
            input.setSpeed(negativeSpeed);
        }
        shooter.updateInputs(input);
    }

    public void runIntake(){
        ShooterInput input = new ShooterInput();
        input.setSpeed(0.5);
    }

    //same as execute for good reasons bc we need execute and initialize
    @Override
    public void initialize() {
        ShooterInput input = new ShooterInput();
        double postiveSpeed = LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed");
        double negativeSpeed = -LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed");  
        if (direction) {

            input.setSpeed(postiveSpeed);
        } else {
            input.setSpeed(negativeSpeed);
        }
        shooter.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

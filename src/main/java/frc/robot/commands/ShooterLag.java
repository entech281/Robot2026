package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterLag extends EntechCommand {

    private final StoppingCounter counter = new StoppingCounter(0.25);
    private final ShooterSubsystem shooterSS;
    private ShooterInput input = new ShooterInput();
    private AngularVelocity speed;

    public ShooterLag(ShooterSubsystem shooterSS, AngularVelocity speed) {
        super(shooterSS);
        this.shooterSS = shooterSS;
        this.speed = speed;
    }

    @Override
    public void end(boolean interrupted) {
        input = new ShooterInput();
        shooterSS.updateInputs(input);
    }

    @Override
    public void execute() {}

    @Override
    public void initialize() {
        input.setSpeed(speed.in(RPM));
        shooterSS.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
    
}

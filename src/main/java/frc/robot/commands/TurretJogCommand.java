package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Moves the turret by a small step while the command is active.
 * Useful for tuning: hold the button to nudge the turret left/right.
 */
public class TurretJogCommand extends EntechCommand {
    private final TurretSubsystem turret;
    private final double stepDegrees;
    private double targetPosition = 0.0;

    /**
     * @param turret the turret subsystem
     * @param stepDegrees positive to move toward + degrees, negative for -
     */
    public TurretJogCommand(TurretSubsystem turret, double stepDegrees) {
        super(turret);
        this.turret = turret;
        this.stepDegrees = stepDegrees;
    }

    @Override
    public void initialize() {
        // Start from current position
        targetPosition = turret.getOutputs().getCurrentPosition();
    }

    @Override
    public void execute() {
        // Increment target position by step
        targetPosition += stepDegrees;
        
        // Apply the target position
        TurretInput in = new TurretInput();
        in.setRequestedPosition(targetPosition);
        turret.updateInputs(in);
    }

    @Override
    public void end(boolean interrupted) {
        // Hold at current target - don't reset to zero
        TurretInput in = new TurretInput();
        in.setRequestedPosition(targetPosition);
        turret.updateInputs(in);
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}
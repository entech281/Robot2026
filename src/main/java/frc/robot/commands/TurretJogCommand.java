package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Moves the turret by a small step while the command is active.
 * Useful for tuning: hold the button to nudge the turret left/right.
 * Step size is defined in RobotConstants.TURRET.TURRET_JOG_STEP_DEGREES
 */
public class TurretJogCommand extends EntechCommand {
    private final TurretSubsystem turret;
    private final int direction; // -1 or 1 to control direction
    private double targetPosition = 0.0;
    private static final double STEP_DEGREES = RobotConstants.TURRET.TURRET_JOG_STEP_DEGREES;

    /**
     * @param turret the turret subsystem
     * @param direction -1 to move left, 1 to move right
     */
    public TurretJogCommand(TurretSubsystem turret, int direction) {
        super(turret);
        this.turret = turret;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        // Start from current position
        targetPosition = turret.getOutputs().getCurrentPosition();
    }

    @Override
    public void execute() {
        // Increment target position by step in the specified direction
        targetPosition += (direction * STEP_DEGREES);
        
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
package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootFromPresetCommand extends EntechCommand {
    
    private final TurretSubsystem turretSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final boolean usePositionA; // true = Position A, false = Position B
    
    private TurretInput turretInput = new TurretInput();
    private HoodInput hoodInput = new HoodInput();
    private ShooterInput shooterInput = new ShooterInput();
    
    /**
     * @param usePositionA true for Position A, false for Position B
     */
    public ShootFromPresetCommand(
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            boolean usePositionA) {
        
        super(turretSubsystem, hoodSubsystem, shooterSubsystem);
        
        this.turretSubsystem = turretSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.usePositionA = usePositionA;
    }
    
    @Override
    public void initialize() {
        // Get preset values based on position
        double turretAngle, hoodAngle, shooterSpeed;
        
        if (usePositionA) {
            turretAngle = RobotConstants.SHOOTING_PRESETS.POS_A_TURRET_ANGLE;
            hoodAngle = RobotConstants.SHOOTING_PRESETS.POS_A_HOOD_ANGLE;
            shooterSpeed = RobotConstants.SHOOTING_PRESETS.POS_A_SHOOTER_SPEED;
        } else {
            turretAngle = RobotConstants.SHOOTING_PRESETS.POS_B_TURRET_ANGLE;
            hoodAngle = RobotConstants.SHOOTING_PRESETS.POS_B_HOOD_ANGLE;
            shooterSpeed = RobotConstants.SHOOTING_PRESETS.POS_B_SHOOTER_SPEED;
        }
        
        // Set turret position
        turretInput.setActivate(true);
        turretInput.setRequestedPosition(turretAngle);
        turretSubsystem.updateInputs(turretInput);
        
        // Set hood position
        hoodInput.setActivate(true);
        hoodInput.setRequestedPosition(hoodAngle);
        hoodSubsystem.updateInputs(hoodInput);
        
        // Set shooter speed
        shooterInput.setSpeed(shooterSpeed);
        shooterSubsystem.updateInputs(shooterInput);
    }
    
    @Override
    public void execute() {
        // Keep updating inputs
        turretSubsystem.updateInputs(turretInput);
        hoodSubsystem.updateInputs(hoodInput);
        shooterSubsystem.updateInputs(shooterInput);
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when all subsystems are at their target positions
        boolean turretReady = turretSubsystem.toOutputs().isAtRequestedPosition();
        boolean hoodReady = hoodSubsystem.toOutputs().isAtRequestedPosition();
        boolean shooterReady = shooterSubsystem.toOutputs().isAtSpeed();
        
        return turretReady && hoodReady && shooterReady;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterInput.setSpeed(0.0);
            shooterSubsystem.updateInputs(shooterInput);
        }
    }
}
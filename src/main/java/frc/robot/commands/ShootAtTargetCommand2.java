package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.LiveTuning;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.TurretCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShootAtTargetCommand2 extends EntechCommand{
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private final TransferSubsystem transferSS;
    private final TurretSubsystem turretSS;
    private ShooterCalculator shooterCalculator;
    private TurretCalculator turretCalculator;
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();
    private TransferInput transferInput = new TransferInput();
    private TurretInput turretInput = new TurretInput();

    public ShootAtTargetCommand2(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem, TurretCalculator turretCalculator, ShooterCalculator shooterCalculator) {
        super(hoodSubsystem, shooterSubsystem, transferSubsystem, turretSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.transferSS = transferSubsystem;
        this.turretSS = turretSubsystem;
        this.shooterCalculator = shooterCalculator;
        this.turretCalculator = turretCalculator;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        ShotDataRange shotRange = shooterCalculator.calculateShot();
        double targetTurretAngle = turretCalculator.calculateTargetTurretAngle();

        ShotData shot = shotRange.getIdealShot();

        shooterInput.setSpeed(shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));
        hoodInput.setRequestedPosition(shot.getHoodAngle().in(Degree));
        turretInput.setRequestedPosition(targetTurretAngle);

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);
        turretSS.updateInputs(turretInput);

        if (shooterCalculator.isValidShot(Degrees.of(hoodSS.getOutputs().getHoodMotor().getCurrentPosition()), RPM.of(shooterSS.getOutputs().getShooterMotorA().getCurrentSpeed()), Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)) && turretCalculator.isValidTurretAngle(turretSS.getOutputs().getTurretMotor().getCurrentPosition(), RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES)) {
            transferInput.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
            transferSS.updateInputs(transferInput);
        }
        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ManualShootCommand extends EntechCommand {
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private final TransferSubsystem transferSS;
    private final TurretSubsystem turretSS;
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();
    private TransferInput transferInput = new TransferInput();
    private TurretInput turretInput = new TurretInput();
    private Angle turretAngle;
    private AngularVelocity shooterSpeed;
    private Angle hoodAngle;
    private boolean moveTurret;

    public ManualShootCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem, Angle turretAngle,
            AngularVelocity shooterSpeed, Angle hoodAngle, boolean moveTurret) {
        super(hoodSubsystem, shooterSubsystem, transferSubsystem, turretSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.transferSS = transferSubsystem;
        this.turretSS = turretSubsystem;
        this.turretAngle = turretAngle;
        this.shooterSpeed = shooterSpeed;
        this.hoodAngle = hoodAngle;
        this.moveTurret = moveTurret;
    }

    public ManualShootCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem, Angle turretAngle,
            AngularVelocity shooterSpeed, Angle hoodAngle) {
        this(shooterSubsystem, hoodSubsystem, transferSubsystem, turretSubsystem, turretAngle, shooterSpeed, hoodAngle,
                true);
    }

    // no sudden ends
    @Override
    public void end(boolean interrupted) {
        transferInput = new TransferInput();
        hoodInput = new HoodInput();
        shooterInput = new ShooterInput();

        transferSS.acceptInputs(transferInput);
        hoodSS.acceptInputs(hoodInput);
        shooterSS.acceptInputs(shooterInput);
    }

    @Override
    public void execute() {

        shooterInput.setSpeed(shooterSpeed.in(RPM));
        hoodInput.setRequestedPosition(hoodAngle.in(Degrees));
        if (moveTurret) {
            turretInput.setRequestedPosition(turretAngle);
            turretSS.acceptInputs(turretInput);
        }

        shooterSS.acceptInputs(shooterInput);
        hoodSS.acceptInputs(hoodInput);

        boolean turretIsReady = turretSS.getOutputs().isAtRequestedPosition();
        boolean hoodIsReady = hoodSS.getOutputs().isAtRequestedPosition();
        boolean shooterIsReady = shooterSS.getOutputs().isAtSpeed();

        boolean isReadyToShoot = turretIsReady && hoodIsReady && shooterIsReady;

        if (isReadyToShoot) {
            transferInput.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
            transferSS.acceptInputs(transferInput);
        } else {
            transferInput.setSpeed(0.0);
            transferSS.acceptInputs(transferInput);
        }

        Logger.recordOutput("TurretIsReadyToShoot", turretIsReady + "");
        Logger.recordOutput("HoodIsReadyToShoot", hoodIsReady + "");
        Logger.recordOutput("ShooterIsReadyToShoot", shooterIsReady + "");
        Logger.recordOutput("IsReadyToShoot", isReadyToShoot + "");

    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

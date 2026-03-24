package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
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
import frc.robot.util.ShotValidator;
import frc.robot.util.TurretCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShootAtTargetCommand extends EntechCommand {
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private final TransferSubsystem transferSS;
    private final TurretSubsystem turretSS;
    private final Pose3d target;
    
    private ShooterCalculator shooterCalculator = new ShooterCalculator();
    private TurretCalculator turretCalculator = new TurretCalculator();
    private ShotValidator shotValidator = new ShotValidator();
    
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();
    private TransferInput transferInput = new TransferInput();
    private TurretInput turretInput = new TurretInput();
    private boolean shooterAtSpeed = false;

    public ShootAtTargetCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem,
            Pose3d target) {
        super(hoodSubsystem, shooterSubsystem, transferSubsystem, turretSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.transferSS = transferSubsystem;
        this.turretSS = turretSubsystem;
        this.target = target;
    }

    // nothing to end
    @Override
    public void end(boolean interrupted) {
        transferInput = new TransferInput();
        hoodInput = new HoodInput();
        shooterInput = new ShooterInput();

        transferSS.updateInputs(transferInput);
        hoodSS.updateInputs(hoodInput);
        shooterSS.updateInputs(shooterInput);

        shooterAtSpeed = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = RobotIO.getInstance().getOdometryPose();
        ChassisSpeeds robotSpeeds = RobotIO.getInstance().getDriveOutput().getSpeeds();
        Pose3d shooterPose3d = new Pose3d(robotPose2d).transformBy(RobotConstants.SHOOTER.SHOT_TRANSFORM);
        Distance wheelRadius = Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS);

        ShotDataRange shotRange = shooterCalculator.calculateShot(shooterPose3d, target, wheelRadius);
        double targetTurretAngle = turretCalculator.calculateTargetTurretAngle(target.toPose2d(), robotPose2d, robotSpeeds);

        ShotData shot = shotRange.getIdealShot();

        shooterInput.setSpeed(shot.getShotAngularVelocity(wheelRadius).in(RPM));
        hoodInput.setRequestedPosition(shot.getHoodAngle().in(Degree));
        turretInput.setRequestedPosition(Degrees.of(targetTurretAngle));

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);
        turretSS.updateInputs(turretInput);

        // boolean turretIsReady = turretCalculatorSupplier.get().isValidTurretAngle(
        // turretSS.getOutputs().getCurrentPosition(),
        // RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES);
        // Triboolean shotIsReady = shooterCalculatorSupplier.get().isValidShot(
        // Degrees.of(hoodSS.getOutputs().getHoodMotor().getCurrentPosition()),
        // RPM.of(shooterSS.getOutputs().getShooterMotorA().getCurrentSpeed()),
        // Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS));

        // Triboolean isReadyToShoot = shotIsReady.fand(Triboolean.of(turretIsReady));

        
        if (!shooterAtSpeed) {
            shooterAtSpeed = shooterSS.getOutputs().isAtSpeed();
        }

        boolean isReadyToShoot = shotValidator.isReadyToShoot(
                turretSS.getOutputs().isAtRequestedPosition(),
                hoodSS.getOutputs().isAtRequestedPosition(),
                shooterAtSpeed);

        if (isReadyToShoot) {
            transferInput.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
            transferSS.updateInputs(transferInput);
        } else {
            transferInput.setSpeed(0.0);
            transferSS.updateInputs(transferInput);
        }

        // Logger.recordOutput("TurretCalculatorIsReadyToShoot", turretIsReady);
        // Logger.recordOutput("ShotIsReady", shotIsReady + "");
        // Logger.recordOutput("IsReadyToShoot", isReadyToShoot + "");
        // Logger.recordOutput("ShotRange", shotRange + "");

    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

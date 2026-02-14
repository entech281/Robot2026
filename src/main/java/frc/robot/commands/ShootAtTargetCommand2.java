package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShootAtTargetCommand2 extends EntechCommand{
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private ShooterCalculator calculator = new ShooterCalculator();
    private ShooterCalculator shooterCalculator;
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();

    public ShootAtTargetCommand2(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, ShooterCalculator calculator) {
        super(hoodSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.shooterCalculator = calculator;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void execute() {
        //TODO: Transfer stuff
        ShotDataRange shotRange = calculator.calculateShot();

        ShotData shot = shotRange.getIdealShot();

        shooterInput.setSpeed(shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));
        hoodInput.setRequestedPosition(shot.getHoodAngle().in(Degree));

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);

        if (calculator.isValidShot(Degrees.of(hoodSS.getOutputs().getHoodMotor().getCurrentPosition()), RPM.of(shooterSS.getOutputs().getShooterMotorA().getCurrentSpeed()), Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)) ) {
            //TODO: Transfer shoot
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

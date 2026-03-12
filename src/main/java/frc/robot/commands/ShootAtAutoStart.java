package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShootAtAutoStart extends EntechCommand{
    private ShooterInput shooterInput = new ShooterInput();
    private Supplier<ShooterCalculator> shooterCalculatorSupplier;
    private ShooterSubsystem SS = new ShooterSubsystem();

    public ShootAtAutoStart(Supplier<ShooterCalculator> scs, ShooterSubsystem SS) {
        shooterCalculatorSupplier = scs;
        this.SS = SS;
    }

    @Override
    public void initialize() {
        //set shooter speed to 3000 rpm
        //set transfer to on
    }

    @Override
    public void execute(){
        ShotDataRange shotRange = shooterCalculatorSupplier.get().calculateShot();
        ShotData shot = shotRange.getIdealShot();

        shooterInput.setSpeed(shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));

        SS.updateInputs(shooterInput);

    }
    
    @Override
    public boolean isFinished() {
        //check if shooter is at speed and if it is, return true, otherwise return false
        return false;
    }
}

package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.sensors.TurretHomeSwitch.TurretHomeSwitch;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HomeTurretCommand extends EntechCommand{

    TurretSubsystem turretSS;
    TurretHomeSwitch homeSwitch;
    
    public HomeTurretCommand (TurretSubsystem turretSubsystem, TurretHomeSwitch homeSwitch) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.homeSwitch = homeSwitch;
    }

    @Override
    public void end(boolean interrupted) {
        turretSS.reset();
    }

    @Override
    public boolean isFinished() {
        return homeSwitch.getOutputs().isPressed();
    }

    

}

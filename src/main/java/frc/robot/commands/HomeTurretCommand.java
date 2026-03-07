package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HomeTurretCommand extends EntechCommand{

    TurretSubsystem turretSS;
    //TODO: implement this entire command with manual home switch
    public HomeTurretCommand (TurretSubsystem turretSubsystem) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        turretSS.reset();
    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {return false;}

    

}

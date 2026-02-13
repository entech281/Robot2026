package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HomeTurretCommand extends EntechCommand{

    TurretSubsystem turretSS;
    
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
        turretSS.setSpeed(-0.2);
    }

    @Override
    public void initialize() {
        turretSS.setSpeed(-0.2);
    }

    @Override
    public boolean isFinished() {
        return RobotIO.getInstance().getTurretOutput().isAtReverseLimit();
    }

    

}

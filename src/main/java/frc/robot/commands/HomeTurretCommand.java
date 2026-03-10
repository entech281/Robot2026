package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.sensors.HomeTurretSwitch.HomeTurretSwitch;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HomeTurretCommand extends EntechCommand{

    TurretSubsystem turretSS;
    HomeTurretSwitch homeTurretSwitch;
    //TODO: implement this entire command with manual home switch
    public HomeTurretCommand (TurretSubsystem turretSubsystem, HomeTurretSwitch homeTurretSwitch) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.homeTurretSwitch = homeTurretSwitch;
    }

    @Override
    public void end(boolean interrupted) {
        turretSS.reset();
    }

    @Override
    public void execute() {}

    @Override
    public void initialize() {}

    @Override
    public boolean isFinished() {
        return homeTurretSwitch.getOutputs().isPressed();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}

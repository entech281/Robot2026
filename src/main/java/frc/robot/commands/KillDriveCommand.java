package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class KillDriveCommand extends EntechCommand {
    private final DriveSubsystem drive;

    public KillDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.kill();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}

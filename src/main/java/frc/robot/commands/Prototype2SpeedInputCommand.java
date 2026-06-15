package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.prototype2.PrototypeInput2;
import frc.robot.subsystems.prototype2.PrototypeSubsystem2;

public class Prototype2SpeedInputCommand extends EntechCommand{
    private final PrototypeSubsystem2 ps2;

    public Prototype2SpeedInputCommand(PrototypeSubsystem2 ps2){
        super(ps2);
        this.ps2 = ps2;
    }
    
    @Override
    public void end(boolean interrupted) {
        PrototypeInput2 pi = new PrototypeInput2();
        pi.setSpeed(0.0);
        ps2.updateInputs(pi);
    }

    @Override
    public void execute() {
        PrototypeInput2 pi = new PrototypeInput2();
        pi.setSpeed(LiveTuningHandler.getInstance().getValue("Speed"));
        ps2.updateInputs(pi);
    }

    @Override
    public void initialize() {
        PrototypeInput2 pi = new PrototypeInput2();
        pi.setSpeed(LiveTuningHandler.getInstance().getValue("Speed"));
        ps2.updateInputs(pi);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

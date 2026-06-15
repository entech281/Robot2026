package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.prototype.PrototypeInput;
import frc.robot.subsystems.prototype.PrototypeSubsystem;

public class PrototypeSpeedInputCommand extends EntechCommand {
    private final PrototypeSubsystem ps;
    

    public PrototypeSpeedInputCommand(PrototypeSubsystem ps){
        super(ps);
        this.ps = ps;
    }
    
    @Override
    public void end(boolean interrupted) {
        PrototypeInput pi = new PrototypeInput();
        pi.setSpeed(0.0);
        ps.updateInputs(pi);
    }

    @Override
    public void execute() {
        PrototypeInput pi = new PrototypeInput();
        pi.setSpeed(LiveTuningHandler.getInstance().getValue("Speed"));
        ps.updateInputs(pi);
    }

    @Override
    public void initialize() {
        PrototypeInput pi = new PrototypeInput();
        pi.setSpeed(LiveTuningHandler.getInstance().getValue("Speed"));
        ps.updateInputs(pi);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

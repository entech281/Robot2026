package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperOutput;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DropHopper extends EntechCommand{
    private final HopperSubsystem hopper;


    public DropHopper(HopperSubsystem hopper) {
        super(hopper);
        this.hopper = hopper;
    }

    @Override
    public void execute(){
        HopperInput input = new HopperInput();
        HopperOutput output = new HopperOutput();
        
        if(!output.isAtLowerLimit()){
            input.setSpeed(1);
        }
        else input.setSpeed(-1);

        hopper.updateInputs(input);
    }

    @Override
    public void initialize(){
        HopperInput input = new HopperInput();
        HopperOutput output = new HopperOutput();
        
        if(!output.isAtLowerLimit()){
            input.setSpeed(1);
        }
        else input.setSpeed(-1);

        hopper.updateInputs(input);     
    }

    @Override
    public void end(boolean interrupted) {
        hopper.updateInputs(new HopperInput());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


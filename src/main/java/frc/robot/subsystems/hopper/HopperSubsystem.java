package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;

public class HopperSubsystem extends EntechSubsystem<HopperInput, HopperOutput> {

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'initialize'");
    }

    @Override
    public boolean isEnabled() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'isEnabled'");
        return false;
    }

    @Override
    public void updateInputs(HopperInput input) {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public Command getTestCommand() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'getTestCommand'");
        return new Command() {
            
        };
    }

    @Override
    protected HopperOutput toOutputs() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
        return new HopperOutput();
    }
    
}

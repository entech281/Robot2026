package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;

public class HoodSubsystem extends EntechSubsystem<HoodInput, HoodOutput> {

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
    public void updateInputs(HoodInput input) {
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
    protected HoodOutput toOutputs() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
        return new HoodOutput();
    }
    
}

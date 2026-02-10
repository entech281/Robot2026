package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;

public class ClimbSubsystem extends EntechSubsystem<ClimbInput, ClimbOutput> {

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
    public void updateInputs(ClimbInput input) {
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
    protected ClimbOutput toOutputs() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
        return new ClimbOutput();
    }
    
}

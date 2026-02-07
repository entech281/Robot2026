package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;

public class HopperSubsystem extends EntechSubsystem<HopperInput, HopperOutput> {
    private static final boolean ENABLED = false;

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initialize'");
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(HopperInput input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected HopperOutput toOutputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
    }

}

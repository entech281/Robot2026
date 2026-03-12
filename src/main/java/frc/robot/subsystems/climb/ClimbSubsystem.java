package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.io.RobotIO;

public class ClimbSubsystem extends EntechSubsystem<ClimbInput, ClimbOutput> {
    private static final boolean ENABLED = false;

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'initialize'");
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(ClimbInput input) {
        RobotIO.processInput(input);
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected ClimbOutput toOutputs() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
        return new ClimbOutput();
    }

}

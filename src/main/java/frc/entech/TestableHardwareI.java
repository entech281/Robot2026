package frc.entech;

import edu.wpi.first.wpilibj2.command.Command;

public interface TestableHardwareI {
    public Command getTestCommand();

    public boolean isEnabled();

    public String getName();
}

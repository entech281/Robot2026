package frc.entech.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.entech.TestableHardwareI;

public abstract class EntechSubsystem<I extends SubsystemInput, R extends SubsystemOutput>
    extends SubsystemBase implements TestableHardwareI {

  protected EntechSubsystem() {
  }

  public abstract void initialize();

  public void updateInputs(I input) {
    input.log();
    acceptInputs(input);
  }

  protected abstract void acceptInputs(I input);

  protected abstract R toOutputs();

  public R getOutputs() {
    R out = toOutputs();
    out.setCurrentCommand(this.getCurrentCommand() + "");
    out.setDefaultCommand(this.getDefaultCommand() + "");
    return out;
  }
}

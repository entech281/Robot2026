package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.hopper.HopperInput;
import frc.robot.subsystems.hopper.HopperSubsystem;

/**
 * Drops the hopper until the lower limit is reached, then raises it for a short duration
 * and stops. Useful for a single-cycle test.
 */
public class DropThenRaiseHopper extends EntechCommand {
  private final HopperSubsystem hopper;
  private final Timer timer = new Timer();
  private enum State { DROPPING, RAISING, DONE }
  private State state = State.DROPPING;
  private final double raiseDurationSeconds;

  public DropThenRaiseHopper(HopperSubsystem hopper) {
    this(hopper, 0.5);
  }

  public DropThenRaiseHopper(HopperSubsystem hopper, double raiseDurationSeconds) {
    super(hopper);
    this.hopper = hopper;
    this.raiseDurationSeconds = raiseDurationSeconds;
  }

  @Override
  public void initialize() {
    state = State.DROPPING;
    timer.reset();
    timer.stop();
    // start dropping
    HopperInput in = new HopperInput();
    in.setSpeed(1);
    hopper.updateInputs(in);
  }

  @Override
  public void execute() {
    var output = RobotIO.getInstance().getHopperOutput();

    if (state == State.DROPPING) {
      boolean atLower = output != null && output.isAtLowerLimit();
      if (atLower) {
        // start raising
        state = State.RAISING;
        timer.reset();
        timer.start();
        HopperInput in = new HopperInput();
        in.setSpeed(-1);
        hopper.updateInputs(in);
      } else {
        // keep dropping
        HopperInput in = new HopperInput();
        in.setSpeed(1);
        hopper.updateInputs(in);
      }
    } else if (state == State.RAISING) {
      if (timer.hasElapsed(raiseDurationSeconds)) {
        state = State.DONE;
        hopper.updateInputs(new HopperInput());
      } else {
        // continue raising
        HopperInput in = new HopperInput();
        in.setSpeed(-1);
        hopper.updateInputs(in);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    hopper.updateInputs(new HopperInput());
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }
}

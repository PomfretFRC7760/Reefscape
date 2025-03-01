package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LiftRollerCommand extends Command {
  private final LiftIntakeRollerSubsystem rollerSubsystem;
  private final BooleanSupplier shouldIntake;
  private final BooleanSupplier shouldJettison;
  private final DoubleSupplier leftTrigger;
  private final DoubleSupplier rightTrigger;
  private final BooleanSupplier manualOverride;

  public LiftRollerCommand(LiftIntakeRollerSubsystem subsystem, BooleanSupplier shouldIntake, BooleanSupplier shouldJettison, BooleanSupplier manualOverride, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    rollerSubsystem = subsystem;
    this.shouldIntake = shouldIntake;
    this.shouldJettison = shouldJettison;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.manualOverride = manualOverride;
    addRequirements(rollerSubsystem);
  }

  @Override
  public void initialize() {
    if (shouldIntake.getAsBoolean() && shouldJettison.getAsBoolean()) {
      rollerSubsystem.stallRoller();
    }
    else if (shouldIntake.getAsBoolean()) {
      rollerSubsystem.runRollerIntake();
    }
    else if (shouldJettison.getAsBoolean()) {
      rollerSubsystem.runRollerJettison();
    } else {
      rollerSubsystem.stopRoller();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
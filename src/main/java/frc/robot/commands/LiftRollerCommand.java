package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;
import java.util.function.BooleanSupplier;

public class LiftRollerCommand extends Command {
  private final LiftIntakeRollerSubsystem rollerSubsystem;
  private final BooleanSupplier shouldIntake;
  private final BooleanSupplier shouldJettison;

  public LiftRollerCommand(LiftIntakeRollerSubsystem subsystem, BooleanSupplier shouldIntake, BooleanSupplier shouldJettison) {
    rollerSubsystem = subsystem;
    this.shouldIntake = shouldIntake;
    this.shouldJettison = shouldJettison;
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
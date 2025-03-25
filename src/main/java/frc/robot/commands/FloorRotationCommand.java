package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeRotationSubsystem;
import java.util.function.DoubleSupplier;

public class FloorRotationCommand extends Command {
    private final FloorIntakeRotationSubsystem intakeSubsystem;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;

    public FloorRotationCommand(FloorIntakeRotationSubsystem intakeSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
            intakeSubsystem.updatePosition();
            if (leftTrigger.getAsDouble() > 0 && rightTrigger.getAsDouble() > 0) {

            }
            else if (leftTrigger.getAsDouble() > 0) {
                intakeSubsystem.retractIntake(leftTrigger.getAsDouble());
            }
            else if (rightTrigger.getAsDouble() > 0) {
                intakeSubsystem.extendIntake(rightTrigger.getAsDouble());
            }
    }
}

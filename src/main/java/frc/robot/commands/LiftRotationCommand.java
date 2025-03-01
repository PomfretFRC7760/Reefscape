package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftIntakeRotationSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class LiftRotationCommand extends Command {
    private final LiftIntakeRotationSubsystem intakeSubsystem;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;
    private final BooleanSupplier overrideLock;

    public LiftRotationCommand(LiftIntakeRotationSubsystem intakeSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, BooleanSupplier overrideLock) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.overrideLock = overrideLock;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!overrideLock.getAsBoolean()) {
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
}

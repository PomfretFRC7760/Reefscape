package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeRotationSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class FloorRotationCommand extends Command {
    private final FloorIntakeRotationSubsystem intakeSubsystem;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;
    private final BooleanSupplier overrideLock;

    public FloorRotationCommand(FloorIntakeRotationSubsystem intakeSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, BooleanSupplier overrideLock) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.overrideLock = overrideLock;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Limit switch position", intakeSubsystem.getLimitSwitchPosition());
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

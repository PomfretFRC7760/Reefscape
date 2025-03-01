package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;

public class UpperIntakeJettison extends Command{
    private final LiftIntakeRollerSubsystem intake;
    public UpperIntakeJettison(LiftIntakeRollerSubsystem intake2) {
        this.intake = intake2;
        addRequirements(intake2);
    }
    @Override
    public void initialize() {
        intake.autoRunRollerJettison();
        System.out.println("jettison!");
    }
}

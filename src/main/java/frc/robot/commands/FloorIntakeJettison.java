package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeRollerSubsystem;

public class FloorIntakeJettison extends Command{
    private final FloorIntakeRollerSubsystem intake4;
    public FloorIntakeJettison(FloorIntakeRollerSubsystem intake3) {
        this.intake4 = intake3;
        addRequirements(intake3);
    }
    @Override
    public void initialize() {
        intake4.autoJettison();
        System.out.println("jettison!");
    }
}

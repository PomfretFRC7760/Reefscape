package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntakeRotationSubsystem;

public class FloorIntakePosition extends Command{
    private final FloorIntakeRotationSubsystem intake4;
    public FloorIntakePosition(FloorIntakeRotationSubsystem intake3) {
        this.intake4 = intake3;
        addRequirements(intake3);
    }
    @Override
    public void initialize() {
        intake4.autoPosition();
        System.out.println("move intake to position!");
    }
}

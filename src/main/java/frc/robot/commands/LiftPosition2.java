package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPosition2 extends Command{
    private final LiftSubsystem lift;
    public LiftPosition2(LiftSubsystem lift) {
        this.lift = lift;
        addRequirements(lift);
    }
    @Override
    public void initialize() {
        lift.setLiftPosition(21.5);
        System.out.println("position 2");
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPosition3 extends Command{
    private final LiftSubsystem lift;
    public LiftPosition3(LiftSubsystem lift) {
        this.lift = lift;
        addRequirements(lift);
    }
    @Override
    public void initialize() {
        lift.setLiftPosition(53);
        System.out.println("position 3");
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
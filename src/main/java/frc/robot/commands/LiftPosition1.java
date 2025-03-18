package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPosition1 extends Command{
    private final LiftSubsystem lift;
    public LiftPosition1(LiftSubsystem lift) {
        this.lift = lift;
        addRequirements(lift);
    }
    @Override
    public void initialize() {
        //lift.setLiftPosition(0.1);
        System.out.println("position 1");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;

public class LiftAndScore extends SequentialCommandGroup {
    private final LiftSubsystem lift;
    private final LiftIntakeRollerSubsystem liftIntakeRoller;
    private final int liftLevel;

    public LiftAndScore(LiftSubsystem lift, LiftIntakeRollerSubsystem liftIntakeRoller, int liftLevel) {
        this.lift = lift;
        this.liftIntakeRoller = liftIntakeRoller;
        this.liftLevel = liftLevel;

        // Build the sequence when the command is constructed
        addCommands(
            createLiftCommand(liftLevel),
            new UpperIntakeJettison(liftIntakeRoller),
            new LiftPosition1(lift) // Reset lift to level 1 after scoring
        );

        addRequirements(lift, liftIntakeRoller);
    }

    private Command createLiftCommand(int level) {
        switch (level) {
            case 1:
                return new LiftPosition1(lift);
            case 2:
                return new LiftPosition2(lift);
            case 3:
                return new LiftPosition3(lift);
            default:
                return new LiftPosition1(lift);
        }
    }
}

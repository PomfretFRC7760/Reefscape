package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GyroSubsystem;

public class GyroCommand extends Command {
    private final GyroSubsystem gyroSubsystem;

    public GyroCommand(GyroSubsystem gyroSubsystem) {
        this.gyroSubsystem = gyroSubsystem;
        addRequirements(this.gyroSubsystem);
    }

    // Public method to reset the gyro from an InstantCommand
    public void resetGyro() {
        gyroSubsystem.gyroReset();
    }

    @Override
    public boolean isFinished() {
        return true; // Since this command will be used in an InstantCommand, it should finish immediately
    }
}

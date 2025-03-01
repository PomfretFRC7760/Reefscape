package frc.robot.commands;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CameraCommand extends Command {
    private final BooleanSupplier cameraSwitch;
    private final CameraSubsystem cameraSubsystem;
    private double cameraSelection = 0;
    public CameraCommand(BooleanSupplier cameraSwitch, CameraSubsystem cameraSubystem) {
        this.cameraSwitch = cameraSwitch;
        this.cameraSubsystem = cameraSubystem;
        addRequirements(this.cameraSubsystem);
    }
@Override
public void execute() {
    if (cameraSwitch.getAsBoolean()) {
        if (cameraSelection == 0) {
            cameraSelection = 1;
        }
        else {
            cameraSelection = 0;
        }
    }
    cameraSubsystem.SwitchCameras(cameraSelection);
}
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraCommand extends Command {
    private final SendableChooser<Integer> cameraChooser = new SendableChooser<>();
    private final CameraSubsystem cameraSubsystem;

    public CameraCommand(CameraSubsystem cameraSubsystem) {
        this.cameraSubsystem = cameraSubsystem;
        
        // Define options for camera selection
        cameraChooser.setDefaultOption("Coral intake camera", 1);
        cameraChooser.addOption("Algae intake camera", 0);

        // Display on SmartDashboard
        SmartDashboard.putData("Camera Selector", cameraChooser);
        
        addRequirements(this.cameraSubsystem);
    }

    @Override
    public void execute() {
        int selectedCamera = cameraChooser.getSelected(); // Get the selected camera index
        cameraSubsystem.SwitchCameras(selectedCamera); // Switch to the selected camera
    }
}

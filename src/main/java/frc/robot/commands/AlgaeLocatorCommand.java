package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeLocatorCommand extends Command{
    private final VisionSubsystem visionSubsystem;
    private Pose2d algaePose;
    public AlgaeLocatorCommand(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        algaePose = visionSubsystem.getAlgaePose();
        SmartDashboard.putString("Algae Pose", algaePose != null ? algaePose.toString() : "Unacceptable");
        
    }

    public Pose2d getAlgaePose(){
        return algaePose;
    }

    public void setPipeline1(){
        visionSubsystem.setPipeline1();
    }
    
}

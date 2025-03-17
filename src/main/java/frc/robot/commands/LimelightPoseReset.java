package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightPoseReset extends Command{
    private final CANDriveSubsystem drivetrain;
    private final VisionSubsystem visionSubsystem;
    private Pose2d limePose;
    public LimelightPoseReset(CANDriveSubsystem drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        limePose = visionSubsystem.limeFpose();
        SmartDashboard.putString("Limelight Pose", limePose != null ? limePose.toString() : "Unacceptable");
        
    }

    public void resetPose() {
            if (limePose != null) {
                drivetrain.resetPose(limePose);
            }
    }
    
}

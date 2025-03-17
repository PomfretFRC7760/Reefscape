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
    private final BooleanSupplier resetPose;
    public LimelightPoseReset(BooleanSupplier resetPose, CANDriveSubsystem drivetrain, VisionSubsystem visionSubsystem) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.resetPose = resetPose;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("BIG ASS WARNING", "CHECK LIMELIGHT TELEMETRY BEFORE RESETTING POSE!!!");
    }

    @Override
    public void execute() {
        Pose2d limePose = visionSubsystem.limeFpose();
        SmartDashboard.putString("Limelight Pose", limePose != null ? limePose.toString() : "Unacceptable");
        if (resetPose.getAsBoolean()){
            if (limePose != null) {
                drivetrain.resetPose(limePose);
            }
        }
    }
    
}

package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.util.AlgaeLocator;

public class VisionSubsystem extends SubsystemBase {

    private final LimeLocalizationSubsystem limeF;
    private final AlgaeLocator algaeLocator;

    // Constructor that takes a CANDriveSubsystem instance
    public VisionSubsystem(CANDriveSubsystem driveSubsystem, AlgaeLocator algaeLocator) {
        this.limeF = new LimeLocalizationSubsystem("", driveSubsystem);
        this.algaeLocator = algaeLocator;
    }

    public Pose2d limeFpose() {
        Optional<Pose2d> limePose = limeF.getPose();
        return limePose.orElse(null);
    }

    public void setPipeline0(){
        LimelightHelpers.setPipelineIndex("",0);
    }

    public void setPipeline1(){
        LimelightHelpers.setPipelineIndex("",1);
    }

    public Pose2d getAlgaePose() {
        return algaeLocator.locateAlgae();
    }

    public boolean validateTarget(){
        return algaeLocator.validateTarget();
    }
}

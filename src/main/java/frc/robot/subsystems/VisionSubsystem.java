package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final LimeLocalizationSubsystem limeF;

    // Constructor that takes a CANDriveSubsystem instance
    public VisionSubsystem(CANDriveSubsystem driveSubsystem) {
        this.limeF = new LimeLocalizationSubsystem("", driveSubsystem);
    }

    private Pose2d limeFpose() {
        Optional<Pose2d> limePose = limeF.getPose();
        return limePose.orElse(null);
    }
    

    public void updateAll() {
        Pose2d pose = limeFpose(); // Call limeFpose and store the result
        SmartDashboard.putString("Limelight Pose", pose != null ? pose.toString() : "null");
    }
    

    // public void updateFromLimeF(){
    //     if(!limeFpose().isEmpty()) sd.swerveDrive.addVisionMeasurement(limeFpose().get(), limeF.time,limeF.getstdev());//add limelight in
    // }
}

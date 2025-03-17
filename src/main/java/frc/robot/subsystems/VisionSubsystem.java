package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final LimeLocalizationSubsystem limeF;

    // Constructor that takes a CANDriveSubsystem instance
    public VisionSubsystem(CANDriveSubsystem driveSubsystem) {
        this.limeF = new LimeLocalizationSubsystem("", driveSubsystem);
    }

    public Pose2d limeFpose() {
        Optional<Pose2d> limePose = limeF.getPose();
        return limePose.orElse(null);
    }
}

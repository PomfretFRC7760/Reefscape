//written by chatgpt and will need to be fixed later
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import java.util.function.BooleanSupplier;

public class AutoAlign extends Command {
    private final CANDriveSubsystem drivetrain;
    private final NetworkTable limelight;
    private final BooleanSupplier runCondition;

    // Camera and target geometry (in meters)
    private static final double CAMERA_HEIGHT = 0.8;    // Height of the Limelight from the ground (m)
    private static final double TARGET_HEIGHT = 1.5;    // Height of the AprilTag (m)
    private static final double TARGET_DISTANCE = 1.0;  // Desired stopping distance from the tag (m)

    // Control tuning constants
    private static final double TX_TOLERANCE = 1.0;     // Degrees tolerance for vision alignment
    private static final double ALIGN_KP = 0.02;        // Proportional gain for vision (tx) alignment
    private static final double DRIVE_KP = 0.3;         // Proportional gain for distance control
    private static final double HEADING_KP = 0.02;      // Proportional gain for gyro heading correction
    private static final double MAX_SPEED = 1.0;        // Max forward speed (m/s)
    private static final double MAX_ROT_SPEED = 2.0;    // Max rotation speed (rad/s)

    public AutoAlign(CANDriveSubsystem drivetrain, BooleanSupplier runCondition) {
        this.drivetrain = drivetrain;
        this.runCondition = runCondition;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Check if the command is allowed to run.
        if (!runCondition.getAsBoolean()) {
            return;
        }

        // Retrieve vision values from the Limelight.
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        double tid = limelight.getEntry("tid").getDouble(-1.0);

        // If no tag is detected, do nothing.
        if (tid == -1) {
            return;
        }

        // Calculate the distance to the tag using trigonometry.
        double angleToTarget = Math.toRadians(ty);
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTarget);

        // Distance control: calculate forward speed based on the error.
        double distanceError = distance - TARGET_DISTANCE;
        double vx = distanceError * DRIVE_KP;
        vx = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, vx));
        if (Math.abs(distanceError) < 0.1) {
            vx = 0;
        }

        // Vision-based rotational correction: center the target (tx should be zero).
        double visionOmega = -tx * ALIGN_KP;

        // Additional heading correction: drive toward a desired heading of 0°.
        double currentHeading = drivetrain.getGyroAngle();
        double headingError = -currentHeading;  // Desired heading is 0.
        double headingOmega = headingError * HEADING_KP;

        // Combine both rotational corrections.
        double omega = visionOmega + headingOmega;
        omega = Math.max(-MAX_ROT_SPEED, Math.min(MAX_ROT_SPEED, omega));

        // Command the drivetrain with a chassis speeds command.
        drivetrain.driveRobotRelative(new ChassisSpeeds(vx, 0, omega));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion when the command ends.
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // If the run condition is no longer met, finish the command.
        if (!runCondition.getAsBoolean()) {
            return true;
        }
        
        // Check if alignment is complete.
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        double angleToTarget = Math.toRadians(ty);
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToTarget);
        double distanceError = Math.abs(distance - TARGET_DISTANCE);

        double currentHeading = drivetrain.getGyroAngle();
        double headingError = Math.abs(currentHeading);

        return (Math.abs(tx) < TX_TOLERANCE) &&
               (distanceError < 0.1) &&
               (headingError < 2.0);
    }
}

package frc.robot.util;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers; // Ensure this is the correct package for LimelightHelpers

public class AlgaeLocator {

    private final RobotContainer r;

    public AlgaeLocator(RobotContainer r) {
        this.r = r;

    }

    public Pose2d locateAlgae() {
        if (!LimelightHelpers.getTV("")) {
            SmartDashboard.putBoolean("target detected", false);
            return null;
        }
        SmartDashboard.putBoolean("target detected", true);
    
        Pose2d robotPose = r.driveSubsystem.getPose();
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        double limelightHeight = 1.0668;
        double limelightMountAngle = 0;
        double angleToTargetRadians = Math.toRadians(limelightMountAngle + ty);
        double distance = limelightHeight / Math.tan(angleToTargetRadians);
        double horizontalAngleRadians = Math.toRadians(tx);
        double robotHeadingRadians = robotPose.getRotation().getRadians();
        double totalAngle = robotHeadingRadians + horizontalAngleRadians;
        double targetX = robotPose.getX() + distance * Math.cos(totalAngle);
        double targetY = robotPose.getY() + distance * Math.sin(totalAngle);
        Pose2d algaePose = new Pose2d(targetX, targetY, new Rotation2d(Math.toRadians(r.driveSubsystem.getGyroAngle() + tx)));
        return algaePose;
    }

    public boolean validateTarget() {
        if (!LimelightHelpers.getTV("")) {
            return false;
        }
        else{
            return true;
        }
    }
}
    

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import java.util.function.BooleanSupplier;
import frc.robot.util.LocationChooser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class DriveCommand extends Command {
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final CANDriveSubsystem driveSubsystem;
  private final BooleanSupplier robotCentric;
  private boolean robotCentricMode = false;
  private boolean lastRobotCentricButtonState = false;
  private final LocationChooser locationChooser;

  public DriveCommand(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier robotCentric, CANDriveSubsystem driveSubsystem, LocationChooser locationChooser) {
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.robotCentric = robotCentric;
    this.driveSubsystem = driveSubsystem;
    this.locationChooser = locationChooser;

    addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    // Toggle robot-centric mode on button press (rising edge detection)
    boolean currentButtonState = robotCentric.getAsBoolean();
    if (currentButtonState && !lastRobotCentricButtonState) {
      robotCentricMode = !robotCentricMode;  // Toggle mode
    }
    lastRobotCentricButtonState = currentButtonState;  // Update last state

    if (robotCentricMode) {
      driveSubsystem.driveRobotCentric(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    } else {
      driveSubsystem.driveRobot(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    }

    // Display selected pose
    Pose2d selectedPose = locationChooser.selectCoralStation();
    SmartDashboard.putString("Selected Robot Pose", 
    (selectedPose != null) ? selectedPose.toString() : "None");

  }

  // This function should be triggered by a button to drive to the selected pose
  public void driveToSelectedPose() {
    Pose2d selectedPose = locationChooser.selectCoralStation();

    // Handle null pose case
    if (selectedPose == null) {
        return; // Exit the method to prevent further execution
    }

    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        selectedPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );

    CommandScheduler.getInstance().schedule(pathfindingCommand);
}


  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

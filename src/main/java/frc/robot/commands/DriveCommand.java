package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutoConfig;
import frc.robot.util.LocationChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.commands.AlgaeLocatorCommand;

public class DriveCommand extends Command {
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final BooleanSupplier robotCentric;
  private final BooleanSupplier abortAuto;
  private final CANDriveSubsystem driveSubsystem;
  private final LocationChooser locationChooser;
  private final AutoConfig autoConfig;
  private final LiftSubsystem liftSubsystem;
  private final LiftIntakeRollerSubsystem liftIntakeRollerSubsystem;

  private final AlgaeLocatorCommand algaeLocatorCommand;
  
  private boolean robotCentricMode = false;
  private boolean lastRobotCentricButtonState = false;
  private Command pathfindingCommand = null; // Store the pathfinding command
  private Command activePathfindingCommand = null; // Store the active pathfinding command
  
  private final PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
  );

  public DriveCommand(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation, 
                      BooleanSupplier robotCentric, BooleanSupplier abortAuto, 
                      CANDriveSubsystem driveSubsystem, LocationChooser locationChooser, 
                      AutoConfig autoConfig, LiftSubsystem liftSubsystem, 
                      LiftIntakeRollerSubsystem liftIntakeRollerSubsystem, AlgaeLocatorCommand algaeLocatorCommand) {
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.robotCentric = robotCentric;
    this.abortAuto = abortAuto;
    this.driveSubsystem = driveSubsystem;
    this.locationChooser = locationChooser;
    this.autoConfig = autoConfig;
    this.liftSubsystem = liftSubsystem;
    this.liftIntakeRollerSubsystem = liftIntakeRollerSubsystem;
    this.algaeLocatorCommand = algaeLocatorCommand;

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

    // Cancel active path command if joystick moves past threshold
    if ((Math.abs(ySpeed.getAsDouble()) > 0.25 || Math.abs(xSpeed.getAsDouble()) > 0.25 || Math.abs(zRotation.getAsDouble()) > 0.25) || abortAuto.getAsBoolean()) {
      autoAbort();
    }

    // Drive based on selected mode
    if (robotCentricMode) {
      driveSubsystem.driveRobotCentric(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    } else {
      driveSubsystem.driveRobot(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    }

    // Display selected pose
    Pose2d selectedPose = locationChooser.selectCoralStation();
    SmartDashboard.putString("Selected Robot Pose", 
    (selectedPose != null) ? selectedPose.toString() : "None");
    SmartDashboard.putBoolean("target found", algaeLocatorCommand.validateTarget());
    Pose2d algaePose = algaeLocatorCommand.getAlgaePose();
    SmartDashboard.putString("Algae Pose", algaePose != null ? algaePose.toString() : "Unacceptable");
  }

  public void driveToSelectedPose() {
    Pose2d selectedPose = locationChooser.selectCoralStation();

    if (selectedPose == null) {
        return;
    }

    pathfindingCommand = createPathfindingCommand(selectedPose);

    activePathfindingCommand = pathfindingCommand.andThen(simulationPoseReset(selectedPose));

    CommandScheduler.getInstance().schedule(activePathfindingCommand);
  }

  public void driveToAlgae(){
    if (algaeLocatorCommand.getAlgaePose() != null) {
      Pose2d algaePose = algaeLocatorCommand.getAlgaePose();
      pathfindingCommand = createPathfindingCommand(algaePose);

      activePathfindingCommand = pathfindingCommand.andThen(simulationPoseReset(algaePose));

      CommandScheduler.getInstance().schedule(activePathfindingCommand);
    }
  }

  // Helper method to create lift commands dynamically
  private Command createLiftCommand(int level) {
    return (level == 2) ? new LiftPosition2(liftSubsystem) : new LiftPosition1(liftSubsystem);
  }

  private Command simulationPoseReset(Pose2d targetPose) {
    if (driveSubsystem.getGyroAngle() == 0.0) {
      return new InstantCommand(() -> driveSubsystem.resetPose(targetPose));
    }
    else {
      return Commands.none();
    }
  }


  // Helper method to create pathfinding commands dynamically
  private Command createPathfindingCommand(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0).andThen(simulationPoseReset(targetPose));
  }

  // Generic auto coral method
  private Command autoCoral(Pose2d coralPose, Pose2d stationPose, int liftLevel, boolean enabled) {
    if (!enabled) return Commands.none();
      if (stationPose == null) {
        Command pathToCoral = createPathfindingCommand(coralPose);
        Command liftCommand = createLiftCommand(liftLevel);
        Command jettisonCommand = new UpperIntakeJettison(liftIntakeRollerSubsystem);
        Command resetLift = new LiftPosition1(liftSubsystem);

        Command coralSequence = pathToCoral
                                  .andThen(liftCommand)
                                  .andThen(jettisonCommand)
                                  .andThen(resetLift);

        return coralSequence;
      }
      else {
        Command pathToCoral = createPathfindingCommand(coralPose);
        Command pathToStation = createPathfindingCommand(stationPose);
        Command liftCommand = createLiftCommand(liftLevel);
        Command jettisonCommand = new UpperIntakeJettison(liftIntakeRollerSubsystem);
        Command resetLift = new LiftPosition1(liftSubsystem);

        Command coralSequence = pathToStation
                                  .andThen(pathToCoral)
                                  .andThen(liftCommand)
                                  .andThen(jettisonCommand)
                                  .andThen(resetLift);

        return coralSequence;
      }
      
  }


  public Command buildFullAutoSequence() {
    Command coral1 = autoCoral(autoConfig.coral1Pose, null, autoConfig.lift1, autoConfig.enable1);
    Command coral2 = autoCoral(autoConfig.coral2Pose, autoConfig.station2Pose, autoConfig.lift2, autoConfig.enable2);
    Command coral3 = autoCoral(autoConfig.coral3Pose, autoConfig.station3Pose, autoConfig.lift3, autoConfig.enable3);
    Command coral4 = autoCoral(autoConfig.coral4Pose, autoConfig.station4Pose, autoConfig.lift4, autoConfig.enable4);
  
    return coral1.andThen(coral2).andThen(coral3).andThen(coral4);
  }

  public void autoAbort() {
    if (activePathfindingCommand != null && !activePathfindingCommand.isFinished()) {
      activePathfindingCommand.cancel();
      pathfindingCommand = null;
      activePathfindingCommand = null;
    }
  }
  

  @Override
  public void end(boolean isInterrupted) {
    if (activePathfindingCommand != null) {
      activePathfindingCommand.cancel();
      pathfindingCommand = null;
      activePathfindingCommand = null;
    }
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}

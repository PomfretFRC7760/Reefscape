// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FloorRollerCommand;
import frc.robot.commands.GyroCommand;
import frc.robot.commands.LiftAndScore;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;
import frc.robot.subsystems.FloorIntakeRollerSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.commands.LiftCommand;
import frc.robot.subsystems.FloorIntakeRotationSubsystem;
import frc.robot.commands.FloorRotationCommand;
import frc.robot.commands.LiftRollerCommand;
import frc.robot.commands.CameraCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.UpperIntakeJettison;
import frc.robot.commands.LimelightPoseReset; 
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutoConfig;
import frc.robot.util.LocationChooser;
import frc.robot.commands.LiftPosition1;
import frc.robot.commands.LiftPosition2;
import frc.robot.commands.LiftPosition3;
import frc.robot.commands.LiftAndScore;
import frc.robot.util.AlgaeLocator;
import frc.robot.commands.AlgaeLocatorCommand;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final AlgaeLocator algaeLocator = new AlgaeLocator(this);
  private final FloorIntakeRollerSubsystem rollerSubsystem = new FloorIntakeRollerSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  private final GyroCommand gyroCommand = new GyroCommand(gyroSubsystem);
  
  public final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem(gyroSubsystem);
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();

  private final FloorIntakeRotationSubsystem floorIntakeRotationSubsystem = new FloorIntakeRotationSubsystem();

  private final LiftIntakeRollerSubsystem liftIntakeRollerSubsystem = new LiftIntakeRollerSubsystem();
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem, algaeLocator);

  public final DriveCommand driveCommand;

  public final AlgaeLocatorCommand algaeLocatorCommand = new AlgaeLocatorCommand(visionSubsystem);


  private final LocationChooser locationChooser = new LocationChooser(this);

  private LimelightPoseReset limelightPoseReset = new LimelightPoseReset(driveSubsystem, visionSubsystem);

  public final AutoConfig autoConfig;
  

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      0);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      1);

  // The autonomous chooser
  private final SendableChooser<Boolean> autoMode = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;

  private Pose2d lastSelectedPose = null;
  

  private final LiftCommand liftCommand = new LiftCommand(() -> driverController.povUp().getAsBoolean(), () -> driverController.povDown().getAsBoolean(), () -> driverController.povLeft().getAsBoolean(), () -> driverController.povRight().getAsBoolean(), () -> operatorController.getRightY(), liftSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Lift position 1", new LiftPosition1(liftSubsystem));
    NamedCommands.registerCommand("Lift position 2", new LiftPosition2(liftSubsystem));
    NamedCommands.registerCommand("Lift position 3", new LiftPosition3(liftSubsystem));

    autoConfig = new AutoConfig(this);

    NamedCommands.registerCommand("Jettison Coral", new UpperIntakeJettison(liftIntakeRollerSubsystem));
    configureBindings();

    driveCommand = new DriveCommand(
        () -> -driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.y().getAsBoolean(), () -> driverController.x().getAsBoolean(),
        driveSubsystem,
        locationChooser, autoConfig, liftSubsystem, liftIntakeRollerSubsystem, algaeLocatorCommand
    );
    driveSubsystem.setDefaultCommand(driveCommand);

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoMode.setDefaultOption("Custom path", true);
    autoMode.addOption("Preplanned path", false);
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoMode);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "RollerCommand" command with a fixed
    // value ejecting the gamepiece while the button is held

    // before

    
      
    
  

    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive

    // Set the default command for the roller subsystem to an instance of
    // RollerCommand with the values provided by the triggers on the operator
    // controller
    SmartDashboard.putData("Reset gyro", new InstantCommand(() -> gyroCommand.resetGyro()));
    liftSubsystem.setDefaultCommand(liftCommand);
    SmartDashboard.putData("Reset lift encoders", new InstantCommand(() -> liftCommand.resetLiftPosition());
    rollerSubsystem.setDefaultCommand(new FloorRollerCommand(rollerSubsystem, () -> operatorController.a().getAsBoolean(), () -> operatorController.b().getAsBoolean(), () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis()));
    floorIntakeRotationSubsystem.setDefaultCommand(new FloorRotationCommand(floorIntakeRotationSubsystem, () -> operatorController.getLeftTriggerAxis(), () -> operatorController.getRightTriggerAxis()));
    liftIntakeRollerSubsystem.setDefaultCommand(new LiftRollerCommand(liftIntakeRollerSubsystem, () -> driverController.a().getAsBoolean(), () -> driverController.b().getAsBoolean()));
    cameraSubsystem.setDefaultCommand(new CameraCommand(cameraSubsystem));
    SmartDashboard.putData("Reset pose with Limelight", new InstantCommand(() -> limelightPoseReset.resetPose()));
    SmartDashboard.putData("L1 score", new InstantCommand(() -> new LiftAndScore(liftSubsystem,liftIntakeRollerSubsystem, 1).schedule()));
    SmartDashboard.putData("L2 score", new InstantCommand(() -> new LiftAndScore(liftSubsystem,liftIntakeRollerSubsystem, 2).schedule()));
    SmartDashboard.putData("L3 score", new InstantCommand(() -> new LiftAndScore(liftSubsystem,liftIntakeRollerSubsystem, 3).schedule()));
    SmartDashboard.putData("Enable lift manual control", new InstantCommand(() -> liftCommand.manualControlSwitch()));
    SmartDashboard.putData("Abort semi-autonomous", new InstantCommand(() -> driveCommand.autoAbort()));
    SmartDashboard.putData("Drive to algae", new InstantCommand(() -> driveCommand.driveToAlgae()));
    SmartDashboard.putData("Apriltag pipeline", new InstantCommand(() -> limelightPoseReset.setPipeline0()));
    SmartDashboard.putData("Neural network pipeline", new InstantCommand(() -> algaeLocatorCommand.setPipeline1()));
  }

  public void updateSelectedPose() {
    Pose2d currentPose = locationChooser.selectCoralStation(); // Get current selection

    if (currentPose != null && !currentPose.equals(lastSelectedPose)) {
        lastSelectedPose = currentPose; // Update last pose
        driveCommand.driveToSelectedPose(); // Trigger drive function
    }
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoMode.getSelected()) {
      Command autoSequence = driveCommand.buildFullAutoSequence();
      return autoSequence;
    }
    else{
      return autoChooser.getSelected();
    }
  }
}

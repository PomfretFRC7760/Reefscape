// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FloorRollerCommand;
import frc.robot.commands.GyroCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LiftIntakeRollerSubsystem;
import frc.robot.subsystems.FloorIntakeRollerSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.commands.LiftCommand;
import frc.robot.subsystems.FloorIntakeRotationSubsystem;
import frc.robot.commands.FloorRotationCommand;
import frc.robot.subsystems.LiftIntakeRotationSubsystem;
import frc.robot.commands.LiftRotationCommand;
import frc.robot.commands.LiftRollerCommand;
import frc.robot.commands.CameraCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.UpperIntakeJettison;
import frc.robot.commands.FloorIntakeJettison;
import frc.robot.commands.FloorIntakePosition;
//import frc.robot.commands.AutoAlign; 
import frc.robot.subsystems.VisionSubsystem;

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

  private final FloorIntakeRollerSubsystem rollerSubsystem = new FloorIntakeRollerSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem(gyroSubsystem);
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();

  private final FloorIntakeRotationSubsystem floorIntakeRotationSubsystem = new FloorIntakeRotationSubsystem();

  private final LiftIntakeRotationSubsystem liftIntakeRotationSubsystem = new LiftIntakeRotationSubsystem();

  private final LiftIntakeRollerSubsystem liftIntakeRollerSubsystem = new LiftIntakeRollerSubsystem();
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);
  

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      0);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      1);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("Floor Intake Position", new FloorIntakePosition(floorIntakeRotationSubsystem));
    NamedCommands.registerCommand("Lift Intake Jettison Coral", new UpperIntakeJettison(liftIntakeRollerSubsystem));
    NamedCommands.registerCommand("Floor Intake Jettison Coral", new FloorIntakeJettison(rollerSubsystem));
    configureBindings();

    

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption

    autoChooser = AutoBuilder.buildAutoChooser();
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
    driveSubsystem.setDefaultCommand(new DriveCommand(
() -> -driverController.getLeftY(), () -> driverController.getLeftX(),
        () -> -driverController.getRightX(), () -> driverController.povUp().getAsBoolean(),
        driveSubsystem, visionSubsystem));

    // Set the default command for the roller subsystem to an instance of
    // RollerCommand with the values provided by the triggers on the operator
    // controller
    gyroSubsystem.setDefaultCommand(new GyroCommand(() -> driverController.x().getAsBoolean(), gyroSubsystem));
    liftSubsystem.setDefaultCommand(new LiftCommand(() -> operatorController.povUp().getAsBoolean(), () -> operatorController.povDown().getAsBoolean(), () -> operatorController.povLeft().getAsBoolean(), () -> operatorController.povRight().getAsBoolean(), () -> operatorController.x().getAsBoolean(), () -> operatorController.getRightY(), liftSubsystem));
    rollerSubsystem.setDefaultCommand(new FloorRollerCommand(rollerSubsystem, () -> operatorController.a().getAsBoolean(), () -> operatorController.b().getAsBoolean(), () -> driverController.y().getAsBoolean(), () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis()));
    floorIntakeRotationSubsystem.setDefaultCommand(new FloorRotationCommand(floorIntakeRotationSubsystem, () -> operatorController.getLeftTriggerAxis(), () -> operatorController.getRightTriggerAxis(), () -> operatorController.y().getAsBoolean()));
    liftIntakeRotationSubsystem.setDefaultCommand(new LiftRotationCommand(liftIntakeRotationSubsystem, () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis(), () -> driverController.y().getAsBoolean()));
    liftIntakeRollerSubsystem.setDefaultCommand(new LiftRollerCommand(liftIntakeRollerSubsystem, () -> driverController.a().getAsBoolean(), () -> driverController.b().getAsBoolean(), () -> operatorController.y().getAsBoolean(), () -> operatorController.getLeftTriggerAxis(), () -> operatorController.getRightTriggerAxis()));
    cameraSubsystem.setDefaultCommand(new CameraCommand(() -> operatorController.x().getAsBoolean(), cameraSubsystem));
    //BooleanSupplier povDownPressed = () -> driverController.povDown().getAsBoolean();
        
    //Trigger povDownTrigger = new Trigger(povDownPressed);
    //povDownTrigger.whileTrue(new AutoAlign(driveSubsystem, povDownPressed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.VisionSubsystem;

// Command to drive the robot with joystick inputs
public class DriveCommand extends Command {
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final CANDriveSubsystem driveSubsystem;
  private final BooleanSupplier robotCentric;
  private boolean robotCentricMode = false;
  private final VisionSubsystem visionSubsystem;

  // Constructor. Runs only once when the command is first created.
  public DriveCommand(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier robotCentric, CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Save parameters to local variables for use later
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.robotCentric = robotCentric;
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(this.driveSubsystem);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    if (robotCentric.getAsBoolean()) {
      if (!robotCentricMode) {
        robotCentricMode = true;
      }
      else if (robotCentricMode) {
        robotCentricMode = false;
      }
    }
    if (robotCentricMode) {
      driveSubsystem.driveRobotCentric(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    }
    else {
    driveSubsystem.driveRobot(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
    }
    visionSubsystem.updateAll();
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}

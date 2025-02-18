// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

// Command to run the robot at 1/2 power for 1 second in autonomous
public class AutoCommand extends Command {
  CANDriveSubsystem driveSubsystem;
  private final double targetX, targetY, targetHeading;

  // Constructor. Runs only once when the command is first created.
  public AutoCommand(CANDriveSubsystem driveSubsystem, double x, double y, double heading) {
    this.driveSubsystem = driveSubsystem;
    this.targetX = x;
    this.targetY = y;
    this.targetHeading = heading;
    
    addRequirements(driveSubsystem);
}


  // Runs each time the command is scheduled. For this command, we handle starting
  // the timer.
  @Override
  public void initialize() {
      driveSubsystem.goToCoordinate(targetX, targetY, targetHeading);
  }


  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return true;
  }
}

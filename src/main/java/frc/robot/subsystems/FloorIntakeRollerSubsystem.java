// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/** Class to run the rollers over CAN */
public class FloorIntakeRollerSubsystem extends SubsystemBase {
  private final VictorSPX leftRollerMotor;
  private final VictorSPX rightRollerMotor;
  private final Timer timer = new Timer();

  public FloorIntakeRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    leftRollerMotor = new VictorSPX(8);
    rightRollerMotor = new VictorSPX(9);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRollerIntake() {
    leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.25);
    rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.25);
  }
  public void stallRoller() {
    leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.07);
    rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0.07);
  }
  public void runRollerJettison() {
    leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -0.25);
    rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -0.25);
  }
  public void stopRoller() {
    leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  }
  public void manualControlJettison(double rightTrigger) {
      leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightTrigger);
      rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightTrigger);
  }
  public void manualControlIntake(double leftTrigger) {
    leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftTrigger);
    rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftTrigger);
}
public void autoJettison() {
  leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -1);
  rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -1);
  timer.reset();
  timer.start();
  while (timer.get() < 2.0) {
    // Wait for 2 seconds
  }
  leftRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  rightRollerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  timer.stop();
  System.out.println("floor jettison command received");
}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Class to run the rollers over CAN */
public class FloorIntakeRollerSubsystem extends SubsystemBase {
  private final SparkMax motor;

  public FloorIntakeRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    motor = new SparkMax(8, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRollerIntake() {
    motor.set(0.25);
  }
  public void stallRoller() {
    motor.set(0.07);
  }
  public void runRollerJettison() {
    motor.set(-0.25);
  }
  public void stopRoller() {
    motor.set(0);
  }
  public void manualControlJettison(double rightTrigger) {
    motor.set(rightTrigger);
  }
  public void manualControlIntake(double leftTrigger) {
    motor.set(leftTrigger);
}
}

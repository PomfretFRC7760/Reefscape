// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Class to run the rollers over CAN */
public class LiftIntakeRollerSubsystem extends SubsystemBase {
  private final SparkMax motor;

  public LiftIntakeRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    motor = new SparkMax(11, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRollerIntake() {
    motor.set(-0.5);
  }
  public void stallRoller() {
    motor.set( -0.30);
  }
  public void runRollerJettison() {
    motor.set( 0.5);
  }
  public void stopRoller() {
    motor.set( 0);
  }
  public void manualControlJettison(double rightTrigger) {
    motor.set(rightTrigger/2);
}
  public void manualControlIntake(double leftTrigger) {
    motor.set(-leftTrigger/2);
  }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class LiftIntakeRotationSubsystem extends SubsystemBase {
    private final SparkMax motor;

    public LiftIntakeRotationSubsystem() {
        motor = new SparkMax(10, MotorType.kBrushed);
    }

    public void extendIntake(double Speed) {
        motor.set(-Speed/2); // Run motor at 50%
    }

    public void retractIntake(double Speed) {
            motor.set(Speed/2); // Run motor in reverse

    }
    
}

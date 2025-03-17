package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class FloorIntakeRotationSubsystem extends SubsystemBase {
    private final SparkMax motor;

    public FloorIntakeRotationSubsystem() {
        motor = new SparkMax(7, MotorType.kBrushless);
    }

    public void extendIntake(double Speed) {
        motor.set(-Speed/2); // Run motor at 50%
    }

    public void retractIntake(double Speed) {
        motor.set(Speed/2); // Run motor in reverse
    }

    public void autoPosition() {
        motor.set(-0.30);
        Timer.delay(0.65);
        motor.set(0);
    }
}
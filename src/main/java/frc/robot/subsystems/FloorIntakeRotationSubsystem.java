package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class FloorIntakeRotationSubsystem extends SubsystemBase {
    private final VictorSPX motor;
    private final DigitalInput limitSwitch;

    public FloorIntakeRotationSubsystem() {
        motor = new VictorSPX(7);
        limitSwitch = new DigitalInput(0);
    }

    public void extendIntake(double Speed) {
        motor.set(ControlMode.PercentOutput, -Speed/2); // Run motor at 50%
    }

    public void retractIntake(double Speed) {
        if (limitSwitch.get()) {
            motor.set(ControlMode.PercentOutput, Speed/2); // Run motor in reverse
        }
    }

    public boolean getLimitSwitchPosition() {
        return limitSwitch.get();
    }

    public void autoPosition() {
        motor.set(ControlMode.PercentOutput, -0.30);
        Timer.delay(0.65);
        motor.set(ControlMode.PercentOutput, 0);
    }
}
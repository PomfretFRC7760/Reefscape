package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FloorIntakeRotationSubsystem extends SubsystemBase {
    private final SparkMax motor;

    private SparkClosedLoopController controller;
    private SparkMaxConfig config;
    private RelativeEncoder encoder;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM;

    public FloorIntakeRotationSubsystem() {
        motor = new SparkMax(7, MotorType.kBrushless);
        controller = motor.getClosedLoopController();
        encoder = motor.getEncoder();
        config = new SparkMaxConfig();

        kP = 0.1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.25;
        kMinOutput = -0.5;
        maxRPM = 5700;

        config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(1.0 / maxRPM)
        .outputRange(kMinOutput, kMaxOutput);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder.setPosition(0);
    }

    public void extendIntake(double Speed) {
        motor.set(-Speed/2); // Run motor at 50%
    }

    public void retractIntake(double Speed) {
        motor.set(Speed/2); // Run motor in reverse
    }

    public void autoPosition(double position) {
        controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void updatePosition() {
        SmartDashboard.putNumber("Algae intake motor position", encoder.getPosition());
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
public class LiftSubsystem extends SubsystemBase {
    private final SparkMax leftLiftMotor;
    private final SparkMax rightLiftMotor;
    private SparkClosedLoopController leftLiftController;
    private SparkClosedLoopController rightLiftController;
    private SparkMaxConfig liftMotorConfig;
    private RelativeEncoder leftLiftEncoder;
    private RelativeEncoder rightLiftEncoder;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM;
    public LiftSubsystem() {
        leftLiftMotor = new SparkMax(5, MotorType.kBrushless);
        rightLiftMotor = new SparkMax(6, MotorType.kBrushless);
        leftLiftController = leftLiftMotor.getClosedLoopController();
        rightLiftController = rightLiftMotor.getClosedLoopController();
        leftLiftEncoder = leftLiftMotor.getEncoder();
        rightLiftEncoder = rightLiftMotor.getEncoder();
        liftMotorConfig = new SparkMaxConfig();

        kP = 0.1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        liftMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        liftMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(1.0 / maxRPM)
        .outputRange(kMinOutput, kMaxOutput);

        leftLiftMotor.configure(liftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightLiftMotor.configure(liftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        leftLiftEncoder.setPosition(0);
        rightLiftEncoder.setPosition(0);

    }
    public void setLiftPosition(double position) {
        leftLiftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightLiftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
        
    
    public void updatePosition() {
        SmartDashboard.putNumber("Left lift motor position", leftLiftEncoder.getPosition());
        SmartDashboard.putNumber("Right lift motor position", rightLiftEncoder.getPosition());
    }

    public void resetPosition() {
        leftLiftEncoder.setPosition(0);
        rightLiftEncoder.setPosition(0);
    }
    public void manualOverrideControl(double speed) {
        leftLiftMotor.set(-speed/2);
        rightLiftMotor.set(-speed/2);

    }
}

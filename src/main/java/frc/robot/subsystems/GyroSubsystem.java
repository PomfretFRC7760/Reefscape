package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroSubsystem extends SubsystemBase {
    private final ADIS16470_IMU gyro;
    private double yawAngle;
public GyroSubsystem() {
    gyro = new ADIS16470_IMU();
}
public void gyroReset() {
    gyro.reset();
}
public void gyroCalibration() {
    gyro.calibrate();
}
public double getGyroAngle() {
    yawAngle = gyro.getAngle(IMUAxis.kZ);
    SmartDashboard.putNumber("Gyro angle", yawAngle);
    return yawAngle;
}
}

//Pomfret smart fridge code
//big thanks to chatgpt for carrying team 7760

package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;


public class Robot extends TimedRobot {
//if it doesn't fit, you got the wrong hole
//screwed past max screw length and snapped the gate driver PCB
  private final SparkFlex rightMotor1 = new SparkFlex(2, MotorType.kBrushless);
  private final SparkFlex rightMotor2 = new SparkFlex(3, MotorType.kBrushless);
  private final SparkFlex leftMotor1 = new SparkFlex(1, MotorType.kBrushless);
  private final PWMSparkMax leftMotor2 = new PWMSparkMax(4);
  
  private final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  private final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private long buttonPressStartTime = 0; 
  private boolean isButtonPressed = false;
  private long lastPrintTime = 0;

  //Don't be like a certain idiot and have the motors in the wrong order and then spend half an hour trying to figure out why it is strafing instead of rotating

  private MecanumDrive mecanumDrive = new MecanumDrive(leftMotor1, leftMotor2, rightMotor1, rightMotor2);

  //current joystick is temu quality, robot configured to use xbox controller for driving instead
  //change later if we get a better joystick
  Joystick stick = new Joystick(0);
  XboxController xstick = new XboxController(1);

  @Override
  public void robotInit() {

    mecanumDrive.setSafetyEnabled(false);
    gyro.calibrate(); // Calibrate the gyro during initialization
    gyro.reset(); // Reset the gyro to start at 0 degrees
    System.out.println(gyro.isConnected());
  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}
  

  @Override
  public void teleopPeriodic() {
    // this controls the robot with the top notch specialty xbox controller

    //joystick is unused, might change later
    double ySpeed = xstick.getLeftY();
    double xSpeed = xstick.getLeftX();
    double zRotation = xstick.getRightX();

    double yawAngle = gyro.getAngle(IMUAxis.kZ);

    // gotta make this a press and hold because my dumbass is gonna press it and accidentally reset the gyro
    if (xstick.getXButton()) {
      if (!isButtonPressed) {
          buttonPressStartTime = System.currentTimeMillis();
          isButtonPressed = true; 
      }

      // 0.5 seconds hold, might make longer if i still manage to accidentally reset it
      if (isButtonPressed && (System.currentTimeMillis() - buttonPressStartTime >= 500)) {
        gyro.calibrate();  
        gyro.reset(); // gyro reset, hopefully this never has to be used during competition
          isButtonPressed = false; 
      }
  } else {
      isButtonPressed = false;
  }
    
    //gotta make sure i'm not a complete jackass
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastPrintTime >= 20) { 
      double chassisSpeed = calculateChassisSpeed();
     // i know it says smartdashboard but use shuffleboard
     SmartDashboard.putNumber("Front Left Speed (RPM) ", leftEncoder1.getVelocity());
      SmartDashboard.putNumber("Right Front Speed (RPM) ", rightEncoder1.getVelocity());
      SmartDashboard.putNumber("Right Rear Speed (RPM) ", rightEncoder2.getVelocity());
  
      SmartDashboard.putNumber("Robot Orientation (deg) (HOLD X TO RESET) ", -yawAngle);

      SmartDashboard.putNumber("Left Stick Y ", ySpeed);
      SmartDashboard.putNumber("Left Stick X ", xSpeed);
      SmartDashboard.putNumber("Right Stick X ", zRotation);
      SmartDashboard.putNumber("Chassis Speed", chassisSpeed);
      lastPrintTime = currentTime;
  }

  Rotation2d gyroRotation = Rotation2d.fromDegrees(-yawAngle);

  // DRIVE!!!!!!@!@!@!@
  mecanumDrive.driveCartesian(ySpeed, -xSpeed, -zRotation, gyroRotation);

  }

 private double calculateChassisSpeed() {
  //i love dimensional analysis
    double rightSpeed1 = Math.abs(rightEncoder1.getVelocity());
    double rightSpeed2 = Math.abs(rightEncoder2.getVelocity());

    double leftSpeed1 = Math.abs(leftEncoder1.getVelocity());
    // double leftSpeed2 = Math.abs(leftEncoder2.getVelocity());

    // temporary, remove later
    double averageSpeedRPM = (leftSpeed1 + rightSpeed1 + rightSpeed2) / 3.0;

    // double averageSpeedRPM = (leftSpeed1 + leftSpeed2 + rightSpeed1 + rightSpeed2) / 4.0;

    // gearbox reduction ratio
    double gearboxReductionRatio = 10.71;
    double wheelRPM = averageSpeedRPM / gearboxReductionRatio;

    double wheelDiameterMeters = 0.1524; // 6 inches in meters
    double wheelCircumferenceMeters = Math.PI * wheelDiameterMeters;

    double speedMetersPerSecond = (wheelRPM / 60.0) * wheelCircumferenceMeters;

    return speedMetersPerSecond;
}

  //put code in here later idk....
  //worlds or bust
  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}




  @Override
  public void testInit() {}



  @Override
  public void testPeriodic() {}



  @Override
  public void simulationInit() {}




  @Override
  public void simulationPeriodic() {}



  }

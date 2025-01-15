//Pomfret smart fridge code
//big thanks to chatgpt for carrying team 7760

package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;

// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.ControlMode;


public class Robot extends TimedRobot {
  private NetworkTable limelight;
  private boolean limelightDetected = false;
  VideoSink server;
  UsbCamera camera1;
  UsbCamera camera2;
  // Locations of the wheels relative to the robot center.
Translation2d m_frontLeftLocation = new Translation2d(0.292, 0.292);
Translation2d m_frontRightLocation = new Translation2d(0.292, -0.292);
Translation2d m_backLeftLocation = new Translation2d(-0.292, 0.292);
Translation2d m_backRightLocation = new Translation2d(-0.292, -0.292);
// Creating my kinematics object using the wheel locations.
MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
//if it doesn't fit, you got the wrong hole
//screwed past max screw length and snapped the gate driver PCB
  private final SparkFlex rightMotor1 = new SparkFlex(2, MotorType.kBrushless);
  private final SparkFlex rightMotor2 = new SparkFlex(3, MotorType.kBrushless);
  private final SparkFlex leftMotor1 = new SparkFlex(1, MotorType.kBrushless);
  private final SparkFlex leftMotor2 = new SparkFlex(4, MotorType.kBrushless);

  //private final SparkMax auxMotor1 = new SparkMax(5,MotorType.kBrushed);
  //private final SparkMax auxMotor2 = new SparkMax(6,MotorType.kBrushed);

  //private final VictorSPX auxMotor3 = new VictorSPX(7);
 // private final VictorSPX auxMotor4 = new VictorSPX(8);
  private final RelativeEncoder rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder rightEncoder2 = rightMotor2.getEncoder();

  private final RelativeEncoder leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder leftEncoder2 = leftMotor2.getEncoder();

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private long buttonPressStartTime = 0; 
  private boolean isButtonPressed = false;
  private long lastPrintTime = 0;
  private double yawAngle = gyro.getAngle(IMUAxis.kZ);
  Rotation2d gyroRotation = Rotation2d.fromDegrees(-yawAngle);
  //private boolean previousBButton = false;
  //private boolean previousAButton = false;
  private boolean bToggleState = false;
  private boolean bButtonPreviousState = false;

  SparkFlexConfig slowConfigInvert = new SparkFlexConfig();
  SparkFlexConfig slowConfig = new SparkFlexConfig();
  SparkFlexConfig fastConfigInvert = new SparkFlexConfig();
  SparkFlexConfig fastConfig = new SparkFlexConfig();
  boolean isModeConfigured=false;

  //Don't be like a certain idiot and have the motors in the wrong order and then spend half an hour trying to figure out why it is strafing instead of rotating

  private MecanumDrive mecanumDrive = new MecanumDrive(leftMotor1, leftMotor2, rightMotor1, rightMotor2);

  //current joystick is temu quality, robot configured to use xbox controller for driving instead
  //change later if we get a better joystick
  Joystick stick = new Joystick(0);
  XboxController xstick = new XboxController(1);
  double gearRatio = 10.71; // Gearbox reduction ratio
  double wheelCircumference = Math.PI * 0.1524; // 6-inch wheels (in meters)
  
  // Convert encoder positions to wheel positions
  double leftWheelPosition1 = leftEncoder1.getPosition() / gearRatio * wheelCircumference;
  double rightWheelPosition1 = rightEncoder1.getPosition() / gearRatio * wheelCircumference;
  double leftWheelPosition2 = leftEncoder2.getPosition() / gearRatio * wheelCircumference;
  double rightWheelPosition2 = rightEncoder2.getPosition() / gearRatio * wheelCircumference;

  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
    m_kinematics,
    gyroRotation,
    new MecanumDriveWheelPositions(
      leftWheelPosition1, rightWheelPosition1, leftWheelPosition2, rightWheelPosition2
    ),
    new Pose2d(5.0, 13.5, new Rotation2d())
  );
  
  @Override
  public void robotInit() {

    mecanumDrive.setSafetyEnabled(false);
    gyro.calibrate(); 
    gyro.reset(); 
    System.out.println(gyro.isConnected());
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    


        slowConfigInvert
          .inverted(true)
          .openLoopRampRate(0.2)
          .closedLoopRampRate(0.2);

        slowConfig
          .openLoopRampRate(0.2)
          .closedLoopRampRate(0.2);

          fastConfigInvert
            .inverted(true)
            .openLoopRampRate(0)
            .closedLoopRampRate(0);
  
          fastConfig
            .openLoopRampRate(0)
            .closedLoopRampRate(0);

            leftMotor1.configure(fastConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            leftMotor2.configure(fastConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            rightMotor1.configure(fastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            rightMotor2.configure(fastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            SmartDashboard.putNumber("Ramp rate", leftMotor2.configAccessor.getOpenLoopRampRate());
  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}
  

  private Pose2d m_pose;

  @Override
  public void teleopPeriodic() {
    
//limelight testing
    double latency = limelight.getEntry("tl").getDouble(-1.0);

      
        if (latency == -1.0) {
          System.out.println("Limelight not detected.");
          limelightDetected = false; 
      } else if (!limelightDetected) {
          
          System.out.println("Limelight detected! Latency: " + latency + " ms");
          limelightDetected = true; 
      }
    double tv = limelight.getEntry("tv").getDouble(0.0); 
    if (tv == 1.0) {
       
        double tid = limelight.getEntry("tid").getDouble(-1.0); 
        
        
        if (tid != -1.0) {
            System.out.println("Detected AprilTag with ID: " + tid);
        }
    }
    if (xstick.getAButton()) {
      // stop robot
      mecanumDrive.stopMotor();
      return;
  }
  double gearRatio = 10.71; // Gearbox reduction ratio
  double wheelCircumference = Math.PI * 0.1524; // 6-inch wheels (in meters)
  double yawAngle = gyro.getAngle(IMUAxis.kZ);
  Rotation2d gyroRotation = Rotation2d.fromDegrees(-yawAngle);
  // Convert encoder positions to wheel positions
  double leftWheelPosition1 = leftEncoder1.getPosition() / gearRatio * wheelCircumference;
  double rightWheelPosition1 = rightEncoder1.getPosition() / gearRatio * wheelCircumference;
  double leftWheelPosition2 = leftEncoder2.getPosition() / gearRatio * wheelCircumference;
  double rightWheelPosition2 = rightEncoder2.getPosition() / gearRatio * wheelCircumference;
  var wheelPositions = new MecanumDriveWheelPositions(
    leftWheelPosition1, rightWheelPosition1, leftWheelPosition2, rightWheelPosition2);
  // Get the rotation of the robot from the gyro.
  var gyroAngle = gyroRotation;
  // Update the pose
  m_pose = m_odometry.update(gyroAngle, wheelPositions);
    // this controls the robot with the top notch specialty xbox controller

    //joystick is unused, might change later
    double ySpeed = xstick.getLeftY();
    double xSpeed = xstick.getLeftX();
    double zRotation = xstick.getRightX();
//     if (Math.abs(zRotation) < 0.075) {
//     zRotation = 0;
// }
    if (xstick.getBButton()){
      isModeConfigured=false;
    }
    boolean bButtonCurrentState = xstick.getBButton();
    // Detect the rising edge (button just pressed)
    if (bButtonCurrentState && !bButtonPreviousState) {
        bToggleState = !bToggleState; // Toggle the state
    }

    // Store the current state as the previous state for the next loop
    bButtonPreviousState = bButtonCurrentState;

    // Example: Use the toggle state for something
    if (bToggleState) {
      if (ySpeed < -0.15) {
        ySpeed = -0.15;
      }
      if (ySpeed > 0.15) {
        ySpeed = 0.15;
      }
      if (xSpeed < -0.15) {
        ySpeed = -0.15;

      }
      if (xSpeed > 0.15) {
        xSpeed = 0.15;
      }
      if (zRotation < -0.15) {
        zRotation = -0.15;
      }
      if (zRotation > 0.15) {
        zRotation = 0.15;
      }
      if (!isModeConfigured){
        leftMotor1.configure(slowConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        leftMotor2.configure(slowConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor1.configure(slowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor2.configure(slowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SmartDashboard.putNumber("Ramp rate", leftMotor2.configAccessor.getOpenLoopRampRate());
        isModeConfigured= true;
      }
      SmartDashboard.putString("SLOW MODE", "ON");
    } else {
      if (!isModeConfigured){
        
        leftMotor1.configure(fastConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      leftMotor2.configure(fastConfigInvert, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      rightMotor1.configure(fastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      rightMotor2.configure(fastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      SmartDashboard.putNumber("Ramp rate", leftMotor2.configAccessor.getOpenLoopRampRate());
      isModeConfigured= true;
      }
      SmartDashboard.putString("SLOW MODE", "OFF");
    }
    

    if (xstick.getYButton()) {
      server.setSource(camera2);
  } else {
      server.setSource(camera1);
  }

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
  //controlAuxMotor();
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
      SmartDashboard.putNumber("Robot Pose X (meters)", m_pose.getX());
      SmartDashboard.putNumber("Robot Pose Y (meters)", m_pose.getY());
      SmartDashboard.putNumber("Robot Pose Heading (degrees)", m_pose.getRotation().getDegrees());  
      lastPrintTime = currentTime;

      
  }

  

  // DRIVE!!!!!!@!@!@!@
  mecanumDrive.driveCartesian(ySpeed, -xSpeed, -zRotation, gyroRotation);

  }

  private double calculateChassisSpeed() {
    // I love dimensional analysis
    double rightSpeed1 = Math.abs(rightEncoder1.getVelocity());
    double rightSpeed2 = Math.abs(rightEncoder2.getVelocity());

    double leftSpeed1 = Math.abs(leftEncoder1.getVelocity());
    double leftSpeed2 = Math.abs(leftEncoder2.getVelocity());

    double averageSpeedRPM = (leftSpeed1 + leftSpeed2 + rightSpeed1 + rightSpeed2) / 4.0;

    // Gearbox reduction ratio
    double gearboxReductionRatio = 10.71;
    double wheelRPM = averageSpeedRPM / gearboxReductionRatio;

    double wheelDiameterMeters = 0.1524; // 6 inches in meters
    double wheelCircumferenceMeters = Math.PI * wheelDiameterMeters;

    double speedMetersPerSecond = (wheelRPM / 60.0) * wheelCircumferenceMeters;

    // Convert meters per second to kilometers per hour
    double speedKilometersPerHour = speedMetersPerSecond * 3.6;

    return speedKilometersPerHour;
}
// private void controlAuxMotor() {

//   boolean currentBButton = xstick.getBButton();
//   boolean currentAButton = xstick.getAButton();


//   if (currentBButton && !previousBButton) {
//       auxMotor1.set(1.0);
//       auxMotor2.set(1.0);
//       auxMotor3.set(ControlMode.PercentOutput, 1.0);
//       auxMotor4.set(ControlMode.PercentOutput, 1.0);
//   }

//   if (currentAButton && !previousAButton) {
//       auxMotor1.set(0.0); 
//       auxMotor2.set(0.0);
//       auxMotor3.set(ControlMode.PercentOutput, 0.0);
//       auxMotor4.set(ControlMode.PercentOutput, 0.0);
//   }


//   previousBButton = currentBButton;
//   previousAButton = currentAButton;
// }

  //put code in here later idk....
  //worlds or bust

  
  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {
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
    }
  }




  @Override
  public void testInit() {}



  @Override
  public void testPeriodic() {}



  @Override
  public void simulationInit() {}




  @Override
  public void simulationPeriodic() {}



  }

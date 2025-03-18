// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;

public class CANDriveSubsystem extends SubsystemBase {
    private final SparkFlex frontLeft;
    private final SparkFlex frontRight;
    private final SparkFlex rearLeft;
    private final SparkFlex rearRight;

    private final GyroSubsystem gyroSubsystem;

    private final MecanumDrive mecanumDrive;
    private final MecanumDriveOdometry odometry;

    // Encoder direction multipliers (allows tuning in code)
    private double frontLeftEncoderMultiplier = 1.0;
    private double frontRightEncoderMultiplier = 1.0;
    private double rearLeftEncoderMultiplier = 1.0;
    private double rearRightEncoderMultiplier = 1.0;

    private Rotation2d fieldCentricGyro;

    // Robot dimensions (adjust to match your bot)
    private static final double ROBOT_WIDTH = 0.560705;  // Meters
    private static final double ROBOT_LENGTH = 0.4713986; // Meters
    private static final double WHEEL_DIAMETER = 0.1524; // 6 inches in meters
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private static final double GEAR_RATIO = 8.46; // 8.46:1 gearbox

    private double poseAngle;

    private SparkClosedLoopController frontLeftController; 
    private SparkClosedLoopController frontRightController;
    private SparkClosedLoopController rearLeftController;
    private SparkClosedLoopController rearRightController;

    private SparkFlexConfig motorConfig;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, targetRPM;
    private RobotConfig config;

    private Field2d fieldMap;

    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new edu.wpi.first.math.geometry.Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),  // Front Left
        new edu.wpi.first.math.geometry.Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2), // Front Right
        new edu.wpi.first.math.geometry.Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2), // Rear Left
        new edu.wpi.first.math.geometry.Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2) // Rear Right
    );

    private Pose2d robotPose = new Pose2d();

    public CANDriveSubsystem(GyroSubsystem gyroSubsystem) {
        // Initialize motors
        frontRight = new SparkFlex(2, MotorType.kBrushless);
        rearRight = new SparkFlex(3, MotorType.kBrushless);
        frontLeft = new SparkFlex(1, MotorType.kBrushless);
        rearLeft = new SparkFlex(4, MotorType.kBrushless);
        this.gyroSubsystem = gyroSubsystem;
        poseAngle = gyroSubsystem.getGyroAngle();
        mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
        mecanumDrive.setSafetyEnabled(false);
        frontLeft.getEncoder().setPosition(0);
        frontRight.getEncoder().setPosition(0);
        rearLeft.getEncoder().setPosition(0);
        rearRight.getEncoder().setPosition(0);
        motorConfig = new SparkFlexConfig();
        RobotConfig config;

        Matrix<N3, N1> initialStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        Matrix<N3, N1> initialVisionMeasurementStdDevs = VecBuilder.fill(0.45, 0.45, 0.45);

        fieldMap = new Field2d();

        frontLeftController = frontLeft.getClosedLoopController();
        frontRightController = frontRight.getClosedLoopController();
        rearLeftController = rearLeft.getClosedLoopController();
        rearRightController = rearRight.getClosedLoopController();

        kP = 6e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 6784;

        motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(1.0 / maxRPM)
        .outputRange(kMinOutput, kMaxOutput);

        frontLeft.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        frontRight.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearLeft.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rearRight.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // Initialize odometry (assumes gyro angle is 0 at start)
        MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(0, 0, 0, 0);
        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(poseAngle), initialWheelPositions);

        try{
            config = RobotConfig.fromGUISettings();
            Pose2d newPose = new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(0.0));
            resetPose(newPose);
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            }
    }

    public void setupPathPlanner() {
        
    }

    public void updateOdometry() {
        // Read raw encoder positions (in rotations)
        double frontLeftRotations = frontLeft.getEncoder().getPosition() * frontLeftEncoderMultiplier;
        double frontRightRotations = frontRight.getEncoder().getPosition() * frontRightEncoderMultiplier;
        double rearLeftRotations = rearLeft.getEncoder().getPosition() * rearLeftEncoderMultiplier;
        double rearRightRotations = rearRight.getEncoder().getPosition() * rearRightEncoderMultiplier;
    
        // Convert motor rotations to wheel rotations (accounting for gear ratio)
        double frontLeftDistance = (frontLeftRotations / GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
        double frontRightDistance = (frontRightRotations / GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
        double rearLeftDistance = (rearLeftRotations / GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
        double rearRightDistance = (rearRightRotations / GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
    
        // Create a MecanumDriveWheelPositions object using the encoder distances
        MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
            -frontLeftDistance, -rearLeftDistance, -frontRightDistance, -rearRightDistance
        );
    
        poseAngle = gyroSubsystem.getGyroAngle();
        robotPose = odometry.update(Rotation2d.fromDegrees(poseAngle), wheelPositions);
    
        // Publish values to SmartDashboard
        SmartDashboard.putNumber("Front Left Encoder", frontLeftDistance);
        SmartDashboard.putNumber("Front Right Encoder", frontRightDistance);
        SmartDashboard.putNumber("Rear Left Encoder", rearLeftDistance);
        SmartDashboard.putNumber("Rear Right Encoder", rearRightDistance);
        SmartDashboard.putString("Robot Pose", robotPose.toString());
        SmartDashboard.putNumber("Robot X Position", robotPose.getX());
        SmartDashboard.putNumber("Robot Y Position", robotPose.getY());
        SmartDashboard.putNumber("Front Left Velocity", -frontLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Front Right Velocity", -frontRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("Rear Left Velocity", -rearLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Rear Right Velocity", -rearRight.getEncoder().getVelocity());
    }
    
    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putString("Robot chassis speeds",getRobotRelativeSpeeds().toString());
        fieldMap.setRobotPose(robotPose);
        SmartDashboard.putData("Field", fieldMap);
    }
    
    public void resetPose(Pose2d newPose) {
        MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(0, 0, 0, 0);
        //gyroSubsystem.gyroReset();
        //gyroSubsystem.gyroCalibration();
        poseAngle = gyroSubsystem.getGyroAngle();
        odometry.resetPosition(Rotation2d.fromDegrees(poseAngle), initialWheelPositions, newPose);
    }
    public Pose2d getPose() {
        return robotPose;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // Convert motor RPM to wheel linear velocity (m/s)
        double frontLeftSpeed = (frontLeft.getEncoder().getVelocity() / (GEAR_RATIO * 60)) * WHEEL_CIRCUMFERENCE;
        double frontRightSpeed = (frontRight.getEncoder().getVelocity() / (GEAR_RATIO * 60)) * WHEEL_CIRCUMFERENCE;
        double rearLeftSpeed = (rearLeft.getEncoder().getVelocity() / (GEAR_RATIO * 60)) * WHEEL_CIRCUMFERENCE;
        double rearRightSpeed = (rearRight.getEncoder().getVelocity() / (GEAR_RATIO * 60)) * WHEEL_CIRCUMFERENCE;
    
        // Create MecanumDriveWheelSpeeds with corrected wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            -frontLeftSpeed, -frontRightSpeed, -rearLeftSpeed, -rearRightSpeed
        );
        poseAngle = gyroSubsystem.getGyroAngle();
        fieldCentricGyro = Rotation2d.fromDegrees(poseAngle);
        // Convert to chassis speeds
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, fieldCentricGyro);
    }

    public ChassisSpeeds getRobotRelativeSpeedsUnaccounted() {
        // Convert motor RPM to wheel linear velocity (m/s)
        double frontLeftSpeed = (frontLeft.getEncoder().getVelocity());
        double frontRightSpeed = (frontRight.getEncoder().getVelocity());
        double rearLeftSpeed = (rearLeft.getEncoder().getVelocity());
        double rearRightSpeed = (rearRight.getEncoder().getVelocity());
    
        // Create MecanumDriveWheelSpeeds with corrected wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            -frontLeftSpeed, -frontRightSpeed, -rearLeftSpeed, -rearRightSpeed
        );
        poseAngle = gyroSubsystem.getGyroAngle();
        fieldCentricGyro = Rotation2d.fromDegrees(poseAngle);
        // Convert to chassis speeds
        ChassisSpeeds robotRelativeSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, fieldCentricGyro);
    }
    
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Get the robot's pose angle from the gyro
        poseAngle = gyroSubsystem.getGyroAngle();
        fieldCentricGyro = Rotation2d.fromDegrees(poseAngle);
        
        // Convert the provided chassis speeds from field-relative to robot-relative
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond, 
            speeds.omegaRadiansPerSecond, 
            fieldCentricGyro
        );
        
        // Convert the robot-relative ChassisSpeeds to wheel speeds using kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotRelativeSpeeds);
        
        // Apply gear ratio, invert all wheel speeds, and convert to RPM
        frontLeftController.setReference((-wheelSpeeds.frontLeftMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE)) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        frontRightController.setReference((-wheelSpeeds.frontRightMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE)) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rearLeftController.setReference((-wheelSpeeds.rearLeftMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE)) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rearRightController.setReference((-wheelSpeeds.rearRightMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE)) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
        // Send data to SmartDashboard for debugging
        SmartDashboard.putNumber("Front Left Setpoint RPM", (-wheelSpeeds.frontLeftMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE * GEAR_RATIO)) * GEAR_RATIO);
        SmartDashboard.putNumber("Front Right Setpoint RPM", (-wheelSpeeds.frontRightMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE * GEAR_RATIO)) * GEAR_RATIO);
        SmartDashboard.putNumber("Rear Left Setpoint RPM", (-wheelSpeeds.rearLeftMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE * GEAR_RATIO)) * GEAR_RATIO);
        SmartDashboard.putNumber("Rear Right Setpoint RPM", (-wheelSpeeds.rearRightMetersPerSecond * 60 / (WHEEL_CIRCUMFERENCE * GEAR_RATIO)) * GEAR_RATIO);
        
        SmartDashboard.putNumber("Front Left Setpoint", -wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("Front Right Setpoint", -wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("Rear Left Setpoint", -wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("Rear Right Setpoint", -wheelSpeeds.rearRightMetersPerSecond);
    }

    public void driveRobotRelativeUnaccounted(ChassisSpeeds speeds) {
        // Get the robot's pose angle from the gyro
        poseAngle = gyroSubsystem.getGyroAngle();
        fieldCentricGyro = Rotation2d.fromDegrees(poseAngle);
        
        // Convert the provided chassis speeds from field-relative to robot-relative
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond, 
            speeds.omegaRadiansPerSecond, 
            fieldCentricGyro
        );
        
        // Convert the robot-relative ChassisSpeeds to wheel speeds using kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(robotRelativeSpeeds);
        
        // Apply gear ratio, invert all wheel speeds, and convert to RPM
        frontLeftController.setReference((-wheelSpeeds.frontLeftMetersPerSecond) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        frontRightController.setReference((-wheelSpeeds.frontRightMetersPerSecond) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rearLeftController.setReference((-wheelSpeeds.rearLeftMetersPerSecond) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rearRightController.setReference((-wheelSpeeds.rearRightMetersPerSecond) * GEAR_RATIO, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
        // Send data to SmartDashboard for debugging
        SmartDashboard.putNumber("Front Left Setpoint RPM", (-wheelSpeeds.frontLeftMetersPerSecond) * GEAR_RATIO);
        SmartDashboard.putNumber("Front Right Setpoint RPM", (-wheelSpeeds.frontRightMetersPerSecond) * GEAR_RATIO);
        SmartDashboard.putNumber("Rear Left Setpoint RPM", (-wheelSpeeds.rearLeftMetersPerSecond) * GEAR_RATIO);
        SmartDashboard.putNumber("Rear Right Setpoint RPM", (-wheelSpeeds.rearRightMetersPerSecond) * GEAR_RATIO);
        
        SmartDashboard.putNumber("Front Left Setpoint", -wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("Front Right Setpoint", -wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("Rear Left Setpoint", -wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("Rear Right Setpoint", -wheelSpeeds.rearRightMetersPerSecond);
    }
    

    // normal teleop drive
    public void driveRobot(double ySpeed, double xSpeed, double zRotation) {
        poseAngle = gyroSubsystem.getGyroAngle();
        fieldCentricGyro = Rotation2d.fromDegrees(poseAngle);
        mecanumDrive.driveCartesian(-ySpeed, -xSpeed, zRotation, fieldCentricGyro);
    }
    
    public void driveRobotCentric(double ySpeed, double xSpeed, double zRotation) {
        mecanumDrive.driveCartesian(-ySpeed, -xSpeed, zRotation);
    }

    public double getGyroAngle() {
        return gyroSubsystem.getGyroAngle();
    }

    public double getRotationSpeed() {
        return gyroSubsystem.getRotationSpeed();
    }
}

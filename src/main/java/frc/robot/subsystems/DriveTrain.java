// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    // Physical hardware.
    private AHRS gyro;    
    private WPI_TalonFX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    private PIDController leftPIDController, rightPIDController;
    private SimpleMotorFeedforward feedForward;
    
    private boolean leftSideInverted;
    private boolean rightSideInverted;
    private boolean gyroReversed;
    private boolean isReversed;

    private DifferentialDrive driveTrain;
    private DifferentialDriveOdometry driveTrainOdo;
    private DifferentialDriveKinematics driveTrainKinema;

    // Arcade drive values.
    private double velocity;
    private double rotation;

    public DriveTrain() {
        // Prepares motors.
        frontLeftMotor = new WPI_TalonFX(Drive.kFrontLeftMotorID);
        frontRightMotor = new WPI_TalonFX(Drive.kFrontRightMotorID);
        backLeftMotor = new WPI_TalonFX(Drive.kBackLeftMotorID);
        backRightMotor = new WPI_TalonFX(Drive.kBackRightMotorID);

        // Resets motors' settings to their defaults.
        setFactory(frontLeftMotor);
        setFactory(backLeftMotor);
        setFactory(backRightMotor);
        setFactory(frontRightMotor);

        // Right motors inverted.
        frontLeftMotor.setInverted(Drive.kLeftInverted);
        backLeftMotor.setInverted(Drive.kLeftInverted);
        frontRightMotor.setInverted(Drive.kRightInverted);
        backRightMotor.setInverted(Drive.kRightInverted);

        // Because of this, you mostly only deal with front motors
        // as the back motors will follow.
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        // Sets minimum time to accelerate to max speed.
        frontLeftMotor.configOpenloopRamp(Drive.kRampInSec);
        frontRightMotor.configOpenloopRamp(Drive.kRampInSec);
        backLeftMotor.configOpenloopRamp(Drive.kRampInSec);
        backRightMotor.configOpenloopRamp(Drive.kRampInSec);
        
        // Prepares gyro.
        // Depending on how the gyro is connected to the robot,
        // what's passed in the constructor will differ.
        // SerialPort.Port.kUSB for USB
        // SPI.Port.kMXP for rio
        gyro = new AHRS(SPI.Port.kMXP);

        // Ensures a blank slate for odometry to be accurate.
        resetEncoders();

        // The in-code respresentation of the physical drive train.
        driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        driveTrain.setSafetyEnabled(false);

        driveTrainKinema = new DifferentialDriveKinematics(Units.inchesToMeters(Drive.kTrackWidth));
        driveTrainOdo = new DifferentialDriveOdometry(getHeading());

        feedForward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        leftPIDController = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        rightPIDController = new PIDController(Drive.kP, Drive.kI, Drive.kD);

        leftSideInverted = Drive.kLeftInverted;
        rightSideInverted = Drive.kRightInverted;
        gyroReversed = Drive.kGyroReversed;
        isReversed = false;
    }

    @Override
    public void periodic() {
        // Puts important and helpful information about the DriveTrain on our dashboard.
        driveTrainOdo.update(getHeading(), getLeftWheelDistance(), getRightWheelDistance());
        SmartDashboard.putNumber("LeftDTVelocity", getDriveWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("RightDTVelocity", getDriveWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("RobotHeading", getHeading().getDegrees());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    }

    public void setApplyArcadeDrive(double velocity, double rotation) {
        this.velocity = velocity;
        this.rotation = rotation;
        driveTrain.arcadeDrive(velocity, rotation);
    }

    public double getArcadeVelocity() {
        return velocity;
    }

    public double getArcadeRotation() {
        return rotation;
    }

    public void setMaxSpeed(double maxSpeed) {
        driveTrain.setMaxOutput(maxSpeed);
    }

    public void setDriveMode(NeutralMode driveMode) {
        frontLeftMotor.setNeutralMode(driveMode);
        frontRightMotor.setNeutralMode(driveMode);
        backLeftMotor.setNeutralMode(driveMode);
        backRightMotor.setNeutralMode(driveMode);
    }

    public void setFactory(WPI_TalonFX motor) {
        motor.configFactoryDefault();
    }

    public void setVolts(double leftVoltage, double rightVoltage) {
        frontLeftMotor.setVoltage(leftVoltage);
        frontRightMotor.setVoltage(rightVoltage);
        driveTrain.feed();
    }

    // Returns wheel speeds in m/s.
    public DifferentialDriveWheelSpeeds getDriveWheelSpeeds() {
        double leftRotationsPerSecond = (double) getLeftEncoderVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;        
        double leftVelocity = leftRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        double rightRotationsPerSecond = (double) getRightEncoderVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;        
        double rightVelocity = rightRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees((gyroReversed) ? -gyro.getAngle() : gyro.getAngle());
    }

    public double getLeftWheelDistance() {
        double leftDistance = ((double) getLeftEncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return leftDistance;
    }

    public double getRightWheelDistance() {
        double rightDistance = ((double) getRightEncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return rightDistance;
    }

    public double getLeftEncoderPosition() {
        return frontLeftMotor.getSelectedSensorPosition();
    }
    
    public double getRightEncoderPosition() {
        return frontRightMotor.getSelectedSensorPosition();
    }
    
    public double getLeftEncoderVelocity() {
        return frontLeftMotor.getSelectedSensorVelocity();
    }
    
    public double getRightEncoderVelocity() {
        return frontRightMotor.getSelectedSensorVelocity();
    }

    public DifferentialDriveKinematics getDriveTrainKinematics() {
        return driveTrainKinema;
    }
    
      public DifferentialDriveOdometry getDriveTrainOdometry() {
        return driveTrainOdo;
    }

    public SimpleMotorFeedforward getSimpleMotorFeedForward() {
        return feedForward;
    }
    
    public PIDController getLeftPIDController() {
        return leftPIDController;
    }
    
    public PIDController getRightPIDController() {
        return rightPIDController;
    }
    
    // Returns the position of the robot in the field.
    public Pose2d getPose() {
        return driveTrainOdo.getPoseMeters();
    }
    
    public double getGyroAngle() {
        return gyroReversed ? -gyro.getAngle() : gyro.getAngle();
    }

    // Reset Functions
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        resetGyro();
        System.out.println("WARNING! " + getHeading());
        driveTrainOdo.resetPosition(pose, getHeading());
    }

    public void resetEncoders() {
        frontLeftMotor.setSelectedSensorPosition(0);
        frontRightMotor.setSelectedSensorPosition(0);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void invertDriveTrain() {
        leftSideInverted = !leftSideInverted;
        rightSideInverted = !rightSideInverted;
        gyroReversed = !gyroReversed;
        frontLeftMotor.setInverted(leftSideInverted);
        backLeftMotor.setInverted(leftSideInverted);
        frontRightMotor.setInverted(rightSideInverted);
        backRightMotor.setInverted(rightSideInverted);
        isReversed = !isReversed;
        if (isReversed) {
            driveTrain = new DifferentialDrive(frontRightMotor, frontLeftMotor);
            driveTrain.setSafetyEnabled(false);
        } else {
            driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
            driveTrain.setSafetyEnabled(false);
        }
    }
}
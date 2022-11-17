// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private AHRS gyro;    
    private WPI_TalonFX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private PIDController leftPIDController, rightPIDController;
    
    private boolean isLeftSideInverted;
    private boolean isRightSideInverted;
    private boolean gyroReversed;
    private boolean isReversed;

    private DifferentialDrive driveTrain;
    private DifferentialDriveOdometry driveTrainOdo;
    private DifferentialDriveKinematics driveTrainKinema;

    private SimpleMotorFeedforward feedForward;

    private double velocity;
    private double rotation;

    public DriveTrain() {
        frontLeftMotor = new WPI_TalonFX(Drive.kFrontLeftMotorID);
        frontRightMotor = new WPI_TalonFX(Drive.kFrontRightMotorID);
        backLeftMotor = new WPI_TalonFX(Drive.kBackLeftMotorID);
        backRightMotor = new WPI_TalonFX(Drive.kBackRightMotorID);
        
        driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        driveTrain.setSafetyEnabled(false);

        feedForward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        leftPIDController = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        rightPIDController = new PIDController(Drive.kP, Drive.kI, Drive.kD);

        setFactory(frontLeftMotor);
        setFactory(backLeftMotor);
        setFactory(backRightMotor);
        setFactory(frontRightMotor);

        frontLeftMotor.setInverted(Drive.kLeftInverted);
        backLeftMotor.setInverted(Drive.kLeftInverted);
        frontRightMotor.setInverted(Drive.kRightInverted);
        backRightMotor.setInverted(Drive.kRightInverted);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setFactory(WPI_TalonFX motor) {
        motor.configFactoryDefault();
    }

    public void setDriveMode(NeutralMode driveMode) {
        frontLeftMotor.setNeutralMode(driveMode);
        frontRightMotor.setNeutralMode(driveMode);
        backLeftMotor.setNeutralMode(driveMode);
        backRightMotor.setNeutralMode(driveMode);
    }

    public void setVolts(double leftVoltage, double rightVoltage) {
        frontLeftMotor.setVoltage(leftVoltage);
        frontRightMotor.setVoltage(rightVoltage);
        driveTrain.feed();
    }

    public void setMaxSpeed(double maxSpeed) {
        driveTrain.setMaxOutput(maxSpeed);
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

    public SimpleMotorFeedforward getSimpleMotorFeedForward() {
        return feedForward;
    }
    
    public PIDController getLeftPIDController() {
        return leftPIDController;
    }
    
    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void resetEncoders() {
        frontLeftMotor.setSelectedSensorPosition(0);
        frontRightMotor.setSelectedSensorPosition(0);
    }
}
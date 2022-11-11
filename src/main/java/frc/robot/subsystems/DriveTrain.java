// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private AHRS gyro;    
    private WPI_TalonFX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private PIDController leftPIDController, rightPIDController;
    
    private boolean isLeftSideInverted;
    private boolean isRightSideInverted;
    private boolean gyroReversed;
    private boolean isReversed;


    public DriveTrain() {
        frontLeftMotor = new WPI_TalonFX(Motor.kFrontLeftMotorID);
        frontRightMotor = new WPI_TalonFX(Motor.kFrontRightMotorID);
        backLeftMotor = new WPI_TalonFX(Motor.kBackLeftMotorID);
        backRightMotor = new WPI_TalonFX(Motor.kBackRightMotorID);
        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
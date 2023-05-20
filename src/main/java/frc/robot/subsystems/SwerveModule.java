// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  // Motors
  private final CANSparkMax m_driveSparkMax;
  private final CANSparkMax m_turnSparkMax;

  // Encoders
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turnEncoder;

  // PID Controllers
  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turnPIDController;

  private final double m_angularOffset;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorId,
    int turnMotorId,
    double angularOffset) {

      // Initialize motors
      m_driveSparkMax = new CANSparkMax(driveMotorId, MotorType.kBrushless);
      m_turnSparkMax = new CANSparkMax(turnMotorId, MotorType.kBrushless);

      // Factory reset to get the controllers to a known state before configuration
      m_driveSparkMax.restoreFactoryDefaults();
      m_turnSparkMax.restoreFactoryDefaults();

      // Configure motors
      m_driveSparkMax.setInverted(ModuleConstants.kDriveMotorInverted);
      m_turnSparkMax.setInverted(ModuleConstants.kTurnMotorInverted);

      m_driveSparkMax.setIdleMode(ModuleConstants.kDriveMotorIdleMode);
      m_turnSparkMax.setIdleMode(ModuleConstants.kTurnMotorIdleMode);

      // Initialize encoders
      m_driveEncoder = m_driveSparkMax.getEncoder();
      m_turnEncoder = m_turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

      // Configure encoders
      m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
      m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);

      m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderPositionFactor);
      m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderVelocityFactor);

      // Initialize PID controllers
      m_drivePIDController = m_driveSparkMax.getPIDController();
      m_turnPIDController = m_turnSparkMax.getPIDController();

      // Configure PID controllers
      m_drivePIDController.setP(ModuleConstants.kDriveP);
      m_drivePIDController.setI(ModuleConstants.kDriveI);
      m_drivePIDController.setD(ModuleConstants.kDriveD);
      m_drivePIDController.setFF(ModuleConstants.kDriveFF);

      m_drivePIDController.setOutputRange(ModuleConstants.kDriveMinOutput, ModuleConstants.kDriveMaxOutput);
      m_drivePIDController.setFeedbackDevice(m_driveEncoder);

      m_turnPIDController.setP(ModuleConstants.kTurnP);
      m_turnPIDController.setI(ModuleConstants.kTurnI);
      m_turnPIDController.setD(ModuleConstants.kTurnD);
      m_turnPIDController.setFF(ModuleConstants.kTurnFF);

      m_turnPIDController.setOutputRange(ModuleConstants.kTurnMinOutput, ModuleConstants.kTurnMaxOutput);
      m_turnPIDController.setFeedbackDevice(m_turnEncoder);

      m_turnPIDController.setPositionPIDWrappingEnabled(true);
      m_turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurnEncoderPositionPIDMinInput);
      m_turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurnEncoderPositionPIDMaxInput);

      // Save controller configuration
      m_driveSparkMax.burnFlash();
      m_turnSparkMax.burnFlash();

      m_angularOffset = angularOffset;
      m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
      m_driveEncoder.setPosition(0);
    }

  /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply the angular offset of the encoder to get the position relative to the robot
    return new SwerveModuleState(m_driveEncoder.getVelocity(),
      new Rotation2d(m_turnEncoder.getPosition() - m_angularOffset));
  }

  /**
   * Returns the current position of the module.
   * 
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply the angular offset of the encoder to get the position relative to the robot
    return new SwerveModulePosition(
      m_driveEncoder.getPosition(),
      new Rotation2d(m_turnEncoder.getPosition() - m_angularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply the angular offset to the desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
      new Rotation2d(m_turnEncoder.getPosition()));

    // Set references for the PID controllers
    m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /**
   * Zeroes all drive encoders.
   */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }
}

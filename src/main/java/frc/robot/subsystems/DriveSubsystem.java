// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Initialize swerve modules
  SwerveModule m_frontLeftModule = new SwerveModule(DriveConstants.kFrontLeftDriveMotorId,
    DriveConstants.kFrontLeftTurnMotorId,
    DriveConstants.kFrontLeftAngularOffset);
  SwerveModule m_frontRightModule = new SwerveModule(DriveConstants.kFrontRightDriveMotorId,
    DriveConstants.kFrontRightTurnMotorId,
    DriveConstants.kFrontRightAngularOffset);
  SwerveModule m_rearLeftModule = new SwerveModule(DriveConstants.kRearLeftDriveMotorId,
    DriveConstants.kRearLeftTurnMotorId,
    DriveConstants.kRearLeftAngularOffset);
  SwerveModule m_rearRightModule = new SwerveModule(DriveConstants.kRearRightDriveMotorId,
    DriveConstants.kRearRightTurnMotorId,
    DriveConstants.kRearRightAngularOffset);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putString("Note", "It's working!");
    // Set all modules to zero
    m_frontLeftModule.setDesiredState(new SwerveModuleState(
      0.0,
      Rotation2d.fromRadians(0)
    ));
    m_frontRightModule.setDesiredState(new SwerveModuleState(
      0.0,
      Rotation2d.fromRadians(0)
    ));
    m_rearLeftModule.setDesiredState(new SwerveModuleState(
      0.0,
      Rotation2d.fromRadians(0)
    ));
    m_rearRightModule.setDesiredState(new SwerveModuleState(
      0.0,
      Rotation2d.fromRadians(0)
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TankDriveSubsystem;

public class TeleopDriveCmd extends CommandBase {
  // Subsystems
  private TankDriveSubsystem m_driveSubsystem;

  // Controllers
  private Joystick m_driverController;

  /** Creates a new TeleopDrive. */
  public TeleopDriveCmd(TankDriveSubsystem driveSubsystem, Joystick driverController) {
    m_driveSubsystem = driveSubsystem;

    m_driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(
      MathUtil.applyDeadband(m_driverController.getY(), OperatorConstants.kDriverControllerDeadband),
      MathUtil.applyDeadband(m_driverController.getZ(), OperatorConstants.kDriverControllerDeadband),
      -m_driverController.getThrottle(),
      m_driverController.getPOV(),
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

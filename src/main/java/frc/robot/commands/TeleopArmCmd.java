// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopArmCmd extends CommandBase {
  // Subsystems
  private ArmSubsystem m_armSubsystem;

  // Controllers
  private XboxController m_operatorController;

  private boolean m_retracting = false;

  /** Creates a new TeleopArmCmd. */
  public TeleopArmCmd(ArmSubsystem armSubsystem, XboxController operatorController) {
    m_armSubsystem = armSubsystem;

    m_operatorController = operatorController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_operatorController.getLeftTriggerAxis() > OperatorConstants.kArmControllerDeadband) {
      m_retracting = true;
    } else {
      m_retracting = false;
    }
    
    m_armSubsystem.operateArm(
      MathUtil.applyDeadband(m_operatorController.getRightY(), OperatorConstants.kArmControllerDeadband),
      m_operatorController.getLeftBumper(),
      m_retracting,
      m_operatorController.getPOV());
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

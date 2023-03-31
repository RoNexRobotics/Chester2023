// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoOneCmd extends CommandBase {
  // Subsystems
  ArmSubsystem m_armSubsystem;

  private final Timer m_autoTimer = new Timer();
  
  /** Creates a new AutoOneCmd. */
  public AutoOneCmd(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoTimer.reset();
    m_autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_autoTimer.get() <= 1) {
      m_armSubsystem.operateArm(1, false, 0, 0);
    } else {
      m_armSubsystem.operateArm(0, false, 0, -1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.operateArm(0, false, 0, -1);
    m_autoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

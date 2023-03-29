// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.ResetDriveEncodersCmd;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.RunVacuumCmd;
import frc.robot.commands.OpenLowerSolenoidCmd;
import frc.robot.commands.OpenUpperSolenoidCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private VacuumSubsystem m_vacuumSubsystem = new VacuumSubsystem();

  // Commands
  private ResetDriveEncodersCmd m_resetDriveEncodersCmd = new ResetDriveEncodersCmd(m_driveSubsystem);
  private ExtendArmCmd m_extendArmCmd = new ExtendArmCmd(m_armSubsystem);
  private RetractArmCmd m_retractArmCmd = new RetractArmCmd(m_armSubsystem);
  private RunVacuumCmd m_runVacuumCmd = new RunVacuumCmd(m_vacuumSubsystem);
  private OpenUpperSolenoidCmd m_openUpperSolenoidCmd = new OpenUpperSolenoidCmd(m_vacuumSubsystem);
  private OpenLowerSolenoidCmd m_openLowerSolenoidCmd = new OpenLowerSolenoidCmd(m_vacuumSubsystem);

  // Controllers
  private Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private XboxController m_armController = new XboxController(OperatorConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_driveSubsystem.drive(
          MathUtil.applyDeadband(
            m_driverController.getY() * DriveConstants.kPowerPercent,
            OperatorConstants.kDriverControllerDeadband),
          MathUtil.applyDeadband(
            m_driverController.getZ() * DriveConstants.kPowerPercent,
            OperatorConstants.kDriverControllerDeadband),
          m_driverController.getThrottle(),
          m_driverController.getPOV(),
          true
        ),
        m_driveSubsystem).withName("Joystick Drive")
    );

    m_armSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_armSubsystem.operateArm(
          MathUtil.applyDeadband(m_armController.getRightY(), OperatorConstants.kArmControllerDeadband),
          m_armController.getPOV()),
        m_armSubsystem).withName("OperateArm")
    );

    m_vacuumSubsystem.setDefaultCommand(
      new InstantCommand(
        () -> m_vacuumSubsystem.vacuumOff(),
        m_vacuumSubsystem).withName("Off")
    );

    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_armSubsystem);
    SmartDashboard.putData(m_vacuumSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset drive encoders
    new JoystickButton(m_driverController, 6).whileTrue(m_resetDriveEncodersCmd);

    // Extend arm
    new JoystickButton(m_armController, XboxController.Button.kY.value).whileTrue(m_extendArmCmd);

    // Retract arm
    new JoystickButton(m_armController, XboxController.Button.kA.value).whileTrue(m_retractArmCmd);

    // Run vacuum
    new JoystickButton(m_armController, XboxController.Button.kRightBumper.value).whileTrue(m_runVacuumCmd);

    // Open upper solenoid
    new JoystickButton(m_armController, XboxController.Button.kX.value).whileTrue(m_openUpperSolenoidCmd);

    // Open lower solenoid
    new JoystickButton(m_armController, XboxController.Button.kB.value).whileTrue(m_openLowerSolenoidCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Use RunCommand for repeating code and InstantCommand for one-time code
    // Use Thread.sleep(milliseconds) surrounded by a try-catch statement to delay the autonomous code
    // return new RunCommand(
      // () -> m_driveSubsystem.trackVisionTarget(),
      // m_driveSubsystem).withName("Auto");
    
    return null;
  }
}

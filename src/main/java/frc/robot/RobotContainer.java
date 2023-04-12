// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoOneCmd;
import frc.robot.commands.CalibrateArmCmd;
import frc.robot.commands.OpenLowerSolenoidCmd;
import frc.robot.commands.OpenUpperSolenoidCmd;
import frc.robot.commands.ResetDriveEncodersCmd;
import frc.robot.commands.RunVacuumCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final VacuumSubsystem m_vacuumSubsystem = new VacuumSubsystem();

  // Commands
  private final ResetDriveEncodersCmd m_resetDriveEncodersCmd = new ResetDriveEncodersCmd(m_driveSubsystem);
  private final CalibrateArmCmd m_calibrateArmCmd = new CalibrateArmCmd(m_armSubsystem);
  private final RunVacuumCmd m_runVacuumCmd = new RunVacuumCmd(m_vacuumSubsystem);
  private final OpenUpperSolenoidCmd m_openUpperSolenoidCmd = new OpenUpperSolenoidCmd(m_vacuumSubsystem);
  private final OpenLowerSolenoidCmd m_openLowerSolenoidCmd = new OpenLowerSolenoidCmd(m_vacuumSubsystem);
  private final AutoOneCmd m_autoOneCmd = new AutoOneCmd(m_armSubsystem, m_driveSubsystem);

  // Controllers
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final XboxController m_armController = new XboxController(OperatorConstants.kArmControllerPort);

  // Shuffleboard
  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    CameraServer.startAutomaticCapture();

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
                false),
            m_driveSubsystem).withName("Joystick Drive"));

    m_armSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_armSubsystem.operateArm(
                MathUtil.applyDeadband(m_armController.getRightY(), OperatorConstants.kArmControllerDeadband),
                m_armController.getLeftBumper(),
                MathUtil.applyDeadband(m_armController.getLeftTriggerAxis(), OperatorConstants.kArmControllerDeadband),
                m_armController.getPOV()),
            m_armSubsystem).withName("OperateArm"));

    m_vacuumSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_vacuumSubsystem.vacuumOff(),
            m_vacuumSubsystem).withName("Off"));

    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Auto 1", m_autoOneCmd);

    SmartDashboard.putData(m_autoChooser);
    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_armSubsystem);
    SmartDashboard.putData(m_vacuumSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset drive encoders
    new JoystickButton(m_driverController, 6).whileTrue(m_resetDriveEncodersCmd);

    // Calibrate arm
    new JoystickButton(m_armController, XboxController.Button.kX.value).whileTrue(m_calibrateArmCmd);

    // Run vacuum
    new JoystickButton(m_armController, XboxController.Button.kRightBumper.value).whileTrue(m_runVacuumCmd);

    // Open upper solenoid
    new JoystickButton(m_armController, XboxController.Button.kA.value).whileTrue(m_openUpperSolenoidCmd);

    // Open lower solenoid
    new JoystickButton(m_armController, XboxController.Button.kY.value).whileTrue(m_openLowerSolenoidCmd);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}

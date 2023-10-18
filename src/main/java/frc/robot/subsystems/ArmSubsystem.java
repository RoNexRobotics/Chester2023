package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private VictorSPX m_raiseMotor;
  private VictorSPX m_extensionMotor;
  private VictorSPX m_pivotMotor;
  private Encoder m_raiseEncoder;
  private Encoder m_extensionEncoder;
  private DigitalInput m_extensionLimitSwitch;
  private DigitalInput m_raiseLimitSwitch;
  private DigitalInput m_pivotUpperLimitSwitch;
  private DigitalInput m_pivotLowerLimitSwitch;
  private boolean m_calibrated = false;

  public ArmSubsystem() {
    m_raiseMotor = new VictorSPX(ArmConstants.kRaiseMotorId);
    m_extensionMotor = new VictorSPX(ArmConstants.kExtensionMotorId);
    m_pivotMotor = new VictorSPX(ArmConstants.kPivotMotorId);

    m_raiseEncoder = new Encoder(
      ArmConstants.kRaiseEncoderChannelA,
      ArmConstants.kRaiseEncoderChannelB,
      ArmConstants.kRaiseEncoderInverted,
      ArmConstants.kRaiseEncoderEncodingType);
    m_extensionEncoder = new Encoder(
      ArmConstants.kExtensionEncoderChannelA,
      ArmConstants.kExtensionEncoderChannelB,
      ArmConstants.kExtensionEncoderInverted,
      ArmConstants.kExtensionEncoderEncodingType);

    m_raiseLimitSwitch = new DigitalInput(ArmConstants.kRaiseLimitSwitchId);
    m_extensionLimitSwitch = new DigitalInput(ArmConstants.kExtensionLimitSwitchId);
    m_pivotUpperLimitSwitch = new DigitalInput(ArmConstants.kPivotUpperLimitSwitchId);
    m_pivotLowerLimitSwitch = new DigitalInput(ArmConstants.kPivotLowerLimitSwitchId);
  }

  @Override
  public void periodic() {
    // Send values to SmartDashboard
    SmartDashboard.putNumber("Raise Encoder Value", m_raiseEncoder.getDistance());
    SmartDashboard.putNumber("Extension Encoder Value", m_extensionEncoder.getDistance());
    SmartDashboard.putBoolean("Arm Calibrated", m_calibrated);
    SmartDashboard.putBoolean("Raise Limit Switch", m_raiseLimitSwitch.get());
    SmartDashboard.putBoolean("Extension Limit Switch", m_extensionLimitSwitch.get());
    SmartDashboard.putBoolean("Pivot Upper Limit Switch", m_pivotUpperLimitSwitch.get());
    SmartDashboard.putBoolean("Pivot Lower Limit Switch", m_pivotLowerLimitSwitch.get());
  }

  public void operateArm(double raisePower, boolean extending, boolean retracting, double pov) {
    SmartDashboard.putNumber("POV", pov);
    double desiredRaiseMotorPower = raisePower * ArmConstants.kRaiseMotorPowerPercent;

    // Raise motor
    if (desiredRaiseMotorPower < 0 && m_raiseEncoder.getDistance() > ArmConstants.kRaiseEncoderMaxValue) { // Up
      m_raiseMotor.set(ControlMode.PercentOutput, desiredRaiseMotorPower);
    } else if (desiredRaiseMotorPower > 0 && !m_raiseLimitSwitch.get()) { // Down
      m_raiseMotor.set(ControlMode.PercentOutput, desiredRaiseMotorPower);
    } else { // Stopped
      m_raiseMotor.set(ControlMode.PercentOutput, 0);
    }

    // Extension motor
    if (extending && m_extensionEncoder.getDistance() < ArmConstants.kExtensionEncoderMaxValue) { // Extend
      m_extensionMotor.set(ControlMode.PercentOutput, ArmConstants.kExtensionMotorPowerPercent);
    } else if (retracting && !m_extensionLimitSwitch.get()) { // Retracting
      m_extensionMotor.set(ControlMode.PercentOutput, -ArmConstants.kExtensionMotorPowerPercent);
    } else { // Stopped
      m_extensionMotor.set(ControlMode.PercentOutput, 0);
    }

    // Pivot motor
    if (pov == 0) { // Up
      SmartDashboard.putString("Note", "Pivot Motor Running Up!");
      // if (!m_pivotUpperLimitSwitch.get()) {
      if (true) {
        m_pivotMotor.set(ControlMode.PercentOutput, -ArmConstants.kPivotMotorPowerPercent);
      } else {
        m_pivotMotor.set(ControlMode.PercentOutput, 0);
      }
    } else if (pov == 180) { // Down
      SmartDashboard.putString("Note", "Pivot Motor Running Down!");
      // TODO: Fix pivot lower limit switch
      // if (!m_pivotLowerLimitSwitch.get()) {
      if (true) {
        m_pivotMotor.set(ControlMode.PercentOutput, ArmConstants.kPivotMotorPowerPercent);
      } else {
        m_pivotMotor.set(ControlMode.PercentOutput, 0);
      }
    } else { // Stopped
      m_pivotMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void calibrate() {
    // Calibrate pivot motor
    // if (!m_pivotLowerLimitSwitch.get()) {
    if (true) {
      m_pivotMotor.set(ControlMode.PercentOutput, -0.6);
    } else {
      m_pivotMotor.set(ControlMode.PercentOutput, 0);
    }

    // Calibrate extension motor
    if (!m_extensionLimitSwitch.get()) {
      m_extensionMotor.set(ControlMode.PercentOutput, -1);
    } else {
      m_extensionMotor.set(ControlMode.PercentOutput, 0);
      m_extensionEncoder.reset();
    }

    // Calibrate raise motor
    if (!m_raiseLimitSwitch.get()) {
      m_raiseMotor.set(ControlMode.PercentOutput, 1);
    } else {
      m_raiseMotor.set(ControlMode.PercentOutput, 0);
      m_raiseEncoder.reset();
      m_calibrated = true;
    }
  }

  public void stopAllMotors() {
    m_raiseMotor.set(ControlMode.PercentOutput, 0);
    m_extensionMotor.set(ControlMode.PercentOutput, 0);
    m_pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isCalibrated() {
    return m_calibrated;
  }
}

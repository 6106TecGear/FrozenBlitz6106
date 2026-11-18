package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Elbow: one NEO on a SPARK Flex, closed-loop position control on the SPARK's own PID.
 *
 * <p>Units: the encoder is converted so everything here is in <b>degrees of the elbow joint</b>.
 * The relative encoder is zeroed at boot to {@link Constants.Elbow#kStartAngleDeg}, so the arm
 * must start each match at that pose.
 *
 * <p>Change the PID at any time with {@link #setPID(double, double, double)} (or with kG:
 * {@link #setPID(double, double, double, double)}), or live from SmartDashboard / Shuffleboard
 * (keys "Elbow/kP", "Elbow/kI", "Elbow/kD", "Elbow/kG").
 */
public class Elbow extends SubsystemBase {

  private final SparkFlex m_motor = new SparkFlex(Constants.Elbow.kMotorId, MotorType.kBrushless);
  private final SparkFlexConfig m_config = new SparkFlexConfig();
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private double m_kP = Constants.Elbow.kP;
  private double m_kI = Constants.Elbow.kI;
  private double m_kD = Constants.Elbow.kD;
  /** Gravity feedforward, VOLTS at horizontal. */
  private double m_kG = Constants.Elbow.kG;
  private double m_maxOutput = Constants.Elbow.kMaxOutput;

  private double m_targetDeg;

  public Elbow() {
    m_config
        .inverted(Constants.Elbow.kInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Elbow.kCurrentLimitAmps);

    // Encoder -> degrees and degrees/second
    m_config.encoder
        .positionConversionFactor(Constants.Elbow.kDegreesPerMotorRotation)
        .velocityConversionFactor(Constants.Elbow.kDegreesPerMotorRotation / 60.0);

    // Soft limits (in degrees, thanks to the conversion above)
    m_config.softLimit
        .forwardSoftLimit(Constants.Elbow.kMaxAngleDeg)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.Elbow.kMinAngleDeg)
        .reverseSoftLimitEnabled(true);

    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    applyPIDToConfig();

    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder.setPosition(Constants.Elbow.kStartAngleDeg);
    m_targetDeg = Constants.Elbow.kStartAngleDeg;

    SmartDashboard.putNumber("Elbow/kP", m_kP);
    SmartDashboard.putNumber("Elbow/kI", m_kI);
    SmartDashboard.putNumber("Elbow/kD", m_kD);
    SmartDashboard.putNumber("Elbow/kG", m_kG);
  }

  // ===================================================================
  //  PID
  // ===================================================================

  private void applyPIDToConfig() {
    m_config.closedLoop
        .pid(m_kP, m_kI, m_kD, ClosedLoopSlot.kSlot0)
        .outputRange(-m_maxOutput, m_maxOutput, ClosedLoopSlot.kSlot0);
  }

  private void pushConfig() {
    applyPIDToConfig();
    // Only touches the changed parameters; does not reset or save to flash
    m_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Changes the elbow position PID gains. Takes effect immediately. */
  public void setPID(double kP, double kI, double kD) {
    m_kP = kP;
    m_kI = kI;
    m_kD = kD;
    pushConfig();
    publishGains();
  }

  /** Same, plus the gravity feedforward kG (volts at horizontal, scaled by cos(angle)). */
  public void setPID(double kP, double kI, double kD, double kG) {
    m_kG = kG;
    setPID(kP, kI, kD);
  }

  /** Limits the PID output power (0..1). Useful while tuning. */
  public void setMaxOutput(double maxOutput) {
    m_maxOutput = MathUtil.clamp(Math.abs(maxOutput), 0.0, 1.0);
    pushConfig();
  }

  /** Puts the PID back to the values in Constants.Elbow. */
  public void resetPIDToDefaults() {
    m_maxOutput = Constants.Elbow.kMaxOutput;
    setPID(Constants.Elbow.kP, Constants.Elbow.kI, Constants.Elbow.kD, Constants.Elbow.kG);
  }

  private void publishGains() {
    SmartDashboard.putNumber("Elbow/kP", m_kP);
    SmartDashboard.putNumber("Elbow/kI", m_kI);
    SmartDashboard.putNumber("Elbow/kD", m_kD);
    SmartDashboard.putNumber("Elbow/kG", m_kG);
  }

  // ===================================================================
  //  CONTROL
  // ===================================================================

  /** Sets the target angle in degrees (clamped to the min/max). The PID holds it. */
  public void setAngle(double degrees) {
    m_targetDeg = MathUtil.clamp(degrees, Constants.Elbow.kMinAngleDeg, Constants.Elbow.kMaxAngleDeg);
  }

  /** Holds wherever the elbow is right now. */
  public void holdCurrentAngle() { m_targetDeg = getAngle(); }

  /** Sets the current position as "degrees" (use when the arm is at a known pose). */
  public void resetAngle(double degrees) {
    m_encoder.setPosition(degrees);
    m_targetDeg = degrees;
  }

  /** Stops moving and holds the current angle with the PID. */
  public void stop() {
    holdCurrentAngle();
  }

  public double getAngle() { return m_encoder.getPosition(); }
  public double getTargetAngle() { return m_targetDeg; }
  public double getVelocityDegPerSec() { return m_encoder.getVelocity(); }

  public boolean atTarget() {
    return Math.abs(getAngle() - m_targetDeg) <= Constants.Elbow.kToleranceDeg;
  }

  // ===================================================================
  //  COMMAND FACTORIES
  // ===================================================================

  /** Goes to an angle and holds it. Ends when it is there. */
  public Command goToCommand(double degrees) {
    return runOnce(() -> setAngle(degrees)).andThen(run(() -> {})).until(this::atTarget);
  }

  public Command stowCommand() { return goToCommand(Constants.Elbow.kStowAngleDeg); }
  public Command midCommand()  { return goToCommand(Constants.Elbow.kMidAngleDeg); }
  public Command highCommand() { return goToCommand(Constants.Elbow.kHighAngleDeg); }

  // ===================================================================
  //  PERIODIC
  // ===================================================================
  @Override
  public void periodic() {
    // Live PID tuning from the dashboard
    double p = SmartDashboard.getNumber("Elbow/kP", m_kP);
    double i = SmartDashboard.getNumber("Elbow/kI", m_kI);
    double d = SmartDashboard.getNumber("Elbow/kD", m_kD);
    double g = SmartDashboard.getNumber("Elbow/kG", m_kG);
    if (p != m_kP || i != m_kI || d != m_kD || g != m_kG) {
      setPID(p, i, d, g);
    }

    // If someone moves the arm while disabled, don't snap back to an old target on enable.
    if (DriverStation.isDisabled()) {
      m_targetDeg = getAngle();
    }

    // Gravity feedforward: strongest when the arm is horizontal (0 deg = horizontal).
    double ffVolts = m_kG * Math.cos(Math.toRadians(getAngle()));
    m_controller.setSetpoint(m_targetDeg, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);

    SmartDashboard.putNumber("Elbow/Angle", getAngle());
    SmartDashboard.putNumber("Elbow/Target", m_targetDeg);
    SmartDashboard.putBoolean("Elbow/AtTarget", atTarget());
    SmartDashboard.putNumber("Elbow/Current", m_motor.getOutputCurrent());
  }
}

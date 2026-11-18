package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber: 2 CIM motors, each driven by a Victor SPX.
 *
 * <p>Two ways to drive it:
 * <ul>
 *   <li><b>Open loop</b> (always works): {@link #setPercent(double)}, -1..1.</li>
 *   <li><b>Position PID</b> (only if a quadrature encoder is wired to the roboRIO DIO and its
 *       channels are set in {@link Constants.Climber}): {@link #setPosition(double)}.</li>
 * </ul>
 * Victors cannot read an encoder themselves, so the PID runs on the roboRIO (WPILib PIDController).
 *
 * <p>Change the PID at any time with {@link #setPID(double, double, double)}, or live from
 * SmartDashboard / Shuffleboard (keys "Climber/kP", "Climber/kI", "Climber/kD").
 */
public class Climber extends SubsystemBase {

  private enum Mode { OPEN_LOOP, POSITION }

  private final VictorSPX m_left = new VictorSPX(Constants.Climber.kLeftVictorId);
  private final VictorSPX m_right = new VictorSPX(Constants.Climber.kRightVictorId);

  /** null when there is no encoder. */
  private final Encoder m_encoder;
  private final PIDController m_pid =
      new PIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD);

  private Mode m_mode = Mode.OPEN_LOOP;
  private double m_percent = 0.0;

  // Last PID values we pushed, to detect dashboard edits
  private double m_lastP, m_lastI, m_lastD;

  public Climber() {
    for (VictorSPX v : new VictorSPX[] {m_left, m_right}) {
      v.configFactoryDefault();
      v.setNeutralMode(NeutralMode.Brake);   // hold the robot up when stopped
    }
    m_left.setInverted(Constants.Climber.kLeftInverted);
    m_right.setInverted(Constants.Climber.kRightInverted);

    if (Constants.Climber.kEncoderChannelA >= 0 && Constants.Climber.kEncoderChannelB >= 0) {
      m_encoder = new Encoder(Constants.Climber.kEncoderChannelA, Constants.Climber.kEncoderChannelB);
      m_encoder.setDistancePerPulse(Constants.Climber.kDistancePerPulse);
    } else {
      m_encoder = null;
    }
    m_pid.setTolerance(Constants.Climber.kPositionToleranceUnits);

    m_lastP = Constants.Climber.kP;
    m_lastI = Constants.Climber.kI;
    m_lastD = Constants.Climber.kD;
    SmartDashboard.putNumber("Climber/kP", m_lastP);
    SmartDashboard.putNumber("Climber/kI", m_lastI);
    SmartDashboard.putNumber("Climber/kD", m_lastD);
  }

  // ===================================================================
  //  PID
  // ===================================================================

  /** Changes the climber position PID gains. Takes effect immediately. */
  public void setPID(double kP, double kI, double kD) {
    m_pid.setPID(kP, kI, kD);
    m_lastP = kP;
    m_lastI = kI;
    m_lastD = kD;
    SmartDashboard.putNumber("Climber/kP", kP);
    SmartDashboard.putNumber("Climber/kI", kI);
    SmartDashboard.putNumber("Climber/kD", kD);
  }

  /** Puts the PID back to the values in Constants.Climber. */
  public void resetPIDToDefaults() {
    setPID(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD);
  }

  // ===================================================================
  //  CONTROL
  // ===================================================================

  /** Direct output -1..1 (no PID). */
  public void setPercent(double percent) {
    m_mode = Mode.OPEN_LOOP;
    m_percent = MathUtil.clamp(percent, -Constants.Climber.kMaxManualOutput,
        Constants.Climber.kMaxManualOutput);
  }

  /** Position PID toward "target" (units of kDistancePerPulse). Ignored if there is no encoder. */
  public void setPosition(double target) {
    if (m_encoder == null) {
      SmartDashboard.putString("Climber/Warning", "No encoder: position PID unavailable");
      return;
    }
    if (m_mode != Mode.POSITION) {
      m_pid.reset();
    }
    m_mode = Mode.POSITION;
    m_pid.setSetpoint(target);
  }

  public void stop() {
    setPercent(0.0);
  }

  public boolean hasEncoder() { return m_encoder != null; }

  public double getPosition() { return m_encoder == null ? 0.0 : m_encoder.getDistance(); }

  public void resetEncoder() { if (m_encoder != null) m_encoder.reset(); }

  public boolean atPosition() {
    return m_mode == Mode.POSITION && m_pid.atSetpoint();
  }

  // ===================================================================
  //  COMMAND FACTORIES
  // ===================================================================

  /** Drive by hand, e.g. climber.manualCommand(() -> joystick.getY()). Stops when released. */
  public Command manualCommand(DoubleSupplier percent) {
    return runEnd(() -> setPercent(percent.getAsDouble()), this::stop);
  }

  /** Goes to a position with PID and holds it until interrupted. */
  public Command goToCommand(double position) {
    return run(() -> setPosition(position));
  }

  public Command stowCommand()     { return goToCommand(Constants.Climber.kStowedPosition); }
  public Command extendCommand()   { return goToCommand(Constants.Climber.kExtendedPosition); }
  public Command climbCommand()    { return goToCommand(Constants.Climber.kClimbedPosition); }

  // ===================================================================
  //  PERIODIC
  // ===================================================================
  @Override
  public void periodic() {
    // Live PID tuning from the dashboard
    double p = SmartDashboard.getNumber("Climber/kP", m_lastP);
    double i = SmartDashboard.getNumber("Climber/kI", m_lastI);
    double d = SmartDashboard.getNumber("Climber/kD", m_lastD);
    if (p != m_lastP || i != m_lastI || d != m_lastD) {
      setPID(p, i, d);
    }

    double output = m_percent;
    if (m_mode == Mode.POSITION && m_encoder != null) {
      output = MathUtil.clamp(m_pid.calculate(m_encoder.getDistance()),
          -Constants.Climber.kMaxManualOutput, Constants.Climber.kMaxManualOutput);
    }
    m_left.set(ControlMode.PercentOutput, output);
    m_right.set(ControlMode.PercentOutput, output);

    SmartDashboard.putNumber("Climber/Output", output);
    SmartDashboard.putNumber("Climber/Position", getPosition());
    SmartDashboard.putString("Climber/Mode", m_mode.name());
  }
}

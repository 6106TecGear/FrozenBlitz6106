package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;

/**
 * Moves the elbow with a controller axis (-1..1), e.g. the joystick Y or a stick on the Xbox pad.
 *
 * <p>The axis does NOT drive the motor directly: it moves the PID target at up to
 * {@link Constants.Elbow#kManualMaxDegPerSec} degrees per second. When you let go of the
 * stick the target stops moving and the PID holds the elbow in place.
 *
 * <p>Usage:
 * <pre>
 *   elbow.setDefaultCommand(new ElbowManualCommand(elbow, () -> pad.rightY()));
 * </pre>
 */
public class ElbowManualCommand extends Command {

  private static final double kLoopSeconds = 0.02;

  private final Elbow m_elbow;
  private final DoubleSupplier m_axis;

  public ElbowManualCommand(Elbow elbow, DoubleSupplier axis) {
    m_elbow = elbow;
    m_axis = axis;
    addRequirements(elbow);
  }

  // No initialize(): the subsystem already keeps its own target (and re-syncs it to the real
  // angle while disabled), so taking over from a preset command never makes the arm jump.

  @Override
  public void execute() {
    double input = MathUtil.applyDeadband(m_axis.getAsDouble(), Constants.Elbow.kStickDeadband);
    if (input != 0.0) {
      m_elbow.setAngle(
          m_elbow.getTargetAngle() + input * Constants.Elbow.kManualMaxDegPerSec * kLoopSeconds);
    }
  }

  /** Never finishes by itself: it is a default / held command. The PID keeps holding the target. */
  @Override
  public boolean isFinished() { return false; }
}

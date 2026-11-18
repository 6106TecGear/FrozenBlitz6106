package frc.robot.commands;

import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

/**
 * Turns the robot until the AprilTag is centered in the Limelight, using the Vision aim PID.
 *
 * <p>The drivetrain is passed in as a function so this works with any swerve you write later,
 * e.g. {@code new AimAtTagCommand(vision, rot -> swerve.drive(0, 0, rot))}. The rotation value is
 * -1..1, counter-clockwise positive (WPILib convention).
 *
 * <p>Hold it on a button ({@code whileTrue}). It ends by itself when aimed.
 * NOTE: this command does not require the Swerve subsystem yet because Swerve is empty;
 * once the swerve exists, pass it in and call addRequirements(swerve).
 */
public class AimAtTagCommand extends Command {

  private final Vision m_vision;
  private final DoubleConsumer m_rotationOutput;

  public AimAtTagCommand(Vision vision, DoubleConsumer rotationOutput) {
    m_vision = vision;
    m_rotationOutput = rotationOutput;
    addRequirements(vision);
  }

  @Override
  public void execute() {
    m_rotationOutput.accept(m_vision.getAimOutput());
  }

  @Override
  public boolean isFinished() { return m_vision.isAimed(); }

  @Override
  public void end(boolean interrupted) {
    m_rotationOutput.accept(0.0);
  }
}

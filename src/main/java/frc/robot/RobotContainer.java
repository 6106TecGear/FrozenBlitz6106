// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.controller;
import frc.robot.commands.Controller;
import frc.robot.commands.ElbowManualCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber m_climber = new Climber();
  private final Elbow m_elbow = new Elbow();
  private final Vision m_vision = new Vision();

  /** Xbox (driver) + Logitech Extreme 3D Pro (operator), both read through our Controller class. */
  private final Controller m_controller =
      new Controller(controller.kDriverControllerPort, controller.kOperatorJoystickPort);

  /** Lets Robot.java use the SAME climber (never create a second Climber: same CAN IDs). */
  public Climber getClimber() { return m_climber; }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate.
   */
  private void configureBindings() {
    // ---------------- ELBOW (Extreme 3D) ----------------
    // Joystick Y moves the elbow target (PID holds it when you let go). Push forward = angle up.
    // If it goes the wrong way, put a minus sign: () -> -m_controller.joyY()
    m_elbow.setDefaultCommand(new ElbowManualCommand(m_elbow, m_controller::joyY));

    // Handle buttons: preset angles
    new Trigger(() -> m_controller.joyButtonHeld(3)).onTrue(m_elbow.stowCommand());
    new Trigger(() -> m_controller.joyButtonHeld(4)).onTrue(m_elbow.midCommand());
    new Trigger(() -> m_controller.joyButtonHeld(5)).onTrue(m_elbow.highCommand());

    // ---------------- CLIMBER (Xbox) ----------------
    // NOTE: no default command on the climber while testing from Robot.teleopPeriodic(),
    // otherwise the scheduler overwrites the test output every loop.
    // Full-speed manual control (right trigger = up, left trigger = down), enable after testing:
    // m_climber.setDefaultCommand(m_climber.manualCommand(m_controller::triggersDifference));

    // Preset positions need an encoder on the climber (see Constants.Climber)
    if (m_climber.hasEncoder()) {
      new Trigger(m_controller::yHeld).onTrue(m_climber.extendCommand());
      new Trigger(m_controller::xHeld).onTrue(m_climber.climbCommand());
      new Trigger(m_controller::bHeld).onTrue(m_climber.stowCommand());
    }

    // ---------------- VISION (Limelight 3) ----------------
    // Once the Swerve has a drive method, hold A to rotate toward the AprilTag:
    // new Trigger(m_controller::aHeld)
    //     .whileTrue(new AimAtTagCommand(m_vision, rot -> m_swerve.drive(0, 0, rot)));
  }
}

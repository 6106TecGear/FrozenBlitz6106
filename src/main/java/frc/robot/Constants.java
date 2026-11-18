// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class controller {
    public static final int kDriverControllerPort = 1;     // Xbox
    public static final int kOperatorJoystickPort = 0;     // Logitech Extreme 3D Pro
  }

  // ===========================================================================
  //  CLIMBER: 2 CIM motors, each on a Victor SPX (CAN)
  //  !! CAN IDs are PLACEHOLDERS - set them to match Phoenix Tuner !!
  // ===========================================================================
  public static class Climber {
    public static final int kLeftVictorId = 1;
    public static final int kRightVictorId = 0;

    /** Flip these if the climber goes the wrong way. */
    public static final boolean kLeftInverted = false;
    public static final boolean kRightInverted = true;   // mirrored on the robot

    /** Max output when driven by hand (safety). */
    public static final double kMaxManualOutput = 1.0;

    // Optional quadrature encoder on the roboRIO DIO ports for position PID.
    // Victors have NO encoder input, so use -1 if you have no encoder (open loop only).
    public static final int kEncoderChannelA = -1;
    public static final int kEncoderChannelB = -1;
    /** Climber travel per encoder pulse, in the unit you want (here: inches of rope/arm). */
    public static final double kDistancePerPulse = 0.01;

    // Default (predetermined) position PID. Output is -1..1, input is the unit above.
    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kPositionToleranceUnits = 0.5;

    // Preset positions (same unit as kDistancePerPulse)
    public static final double kStowedPosition = 0.0;
    public static final double kExtendedPosition = 20.0;
    public static final double kClimbedPosition = 2.0;
  }

  // ===========================================================================
  //  ELBOW: 1 NEO on a SPARK Flex
  //  !! CAN ID is a PLACEHOLDER !!
  // ===========================================================================
  public static class Elbow {
    public static final int kMotorId = 5;
    public static final boolean kInverted = false;
    public static final int kCurrentLimitAmps = 40;

    /** Motor rotations per ONE rotation of the elbow joint (gearbox / chain reduction). */
    public static final double kGearRatio = 100.0;
    /** Degrees of elbow movement per motor rotation (computed). */
    public static final double kDegreesPerMotorRotation = 360.0 / kGearRatio;

    /** The elbow MUST start every match at this angle: the NEO encoder is zeroed at boot. */
    public static final double kStartAngleDeg = 0.0;
    public static final double kMinAngleDeg = 0.0;
    public static final double kMaxAngleDeg = 120.0;

    /** Manual control: how fast the stick moves the target. */
    public static final double kManualMaxDegPerSec = 60.0;
    public static final double kStickDeadband = 0.1;

    // Default (predetermined) position PID. Input = degrees, output = -1..1 duty cycle.
    // kP 0.02 -> full power at 50 deg of error. START LOW, then tune on the real robot.
    public static final double kP = 0.02;
    public static final double kI = 0.0;
    public static final double kD = 0.001;
    /** Gravity feedforward in VOLTS at horizontal (applied as kG * cos(angle); 0 deg = horizontal). */
    public static final double kG = 0.0;
    public static final double kMaxOutput = 0.5;   // limits PID power while tuning
    public static final double kToleranceDeg = 2.0;

    // Preset angles
    public static final double kStowAngleDeg = 0.0;
    public static final double kMidAngleDeg = 60.0;
    public static final double kHighAngleDeg = 110.0;
  }

  // ===========================================================================
  //  LIMELIGHT 3 (AprilTags)
  // ===========================================================================
  public static class Vision {
    /** Must match the camera hostname/name set in the Limelight web UI (default "limelight"). */
    public static final String kLimelightName = "limelight";
    public static final int kAprilTagPipeline = 0;

    // Camera position relative to the robot center (meters / degrees).
    // Forward +, Left +, Up +. Roll/Pitch/Yaw in degrees. MEASURE YOURS!
    public static final double kCamForwardM = 0.30;
    public static final double kCamSideM = 0.0;
    public static final double kCamUpM = 0.50;
    public static final double kCamRollDeg = 0.0;
    public static final double kCamPitchDeg = 25.0;
    public static final double kCamYawDeg = 0.0;

    /** Used for the distance-from-ty estimate. Height of the AprilTag center (meters). */
    public static final double kTagHeightM = 1.45;

    // Default aim PID: input = tx in degrees, output = rotation command -1..1
    public static final double kAimP = 0.03;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0.0015;
    public static final double kAimToleranceDeg = 1.5;
    public static final double kAimMaxOutput = 0.6;

    // Default range PID: input = distance error in meters, output = forward command -1..1
    public static final double kRangeP = 0.8;
    public static final double kRangeI = 0.0;
    public static final double kRangeD = 0.0;

    // Reject pose estimates that are likely bad.
    public static final double kMaxTagDistanceM = 4.0;
    public static final double kMaxYawRateDegPerSec = 720.0;
  }
}

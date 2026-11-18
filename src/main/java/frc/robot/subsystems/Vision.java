package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Vision: Limelight 3 running the AprilTag (fiducial) pipeline.
 *
 * <p>Provides:
 * <ul>
 *   <li>Basic target data: {@link #hasTarget()}, {@link #getTagId()}, {@link #getTx()}, ...</li>
 *   <li>Distance to the tag from the camera angle: {@link #getDistanceToTagM()}</li>
 *   <li>Aim / range PID helpers for driving toward a tag: {@link #getAimOutput()},
 *       {@link #getRangeOutput(double)}</li>
 *   <li>Field pose with MegaTag2: {@link #getPoseEstimate(double, double)}, ready to feed a
 *       SwerveDrivePoseEstimator.addVisionMeasurement(...) once the Swerve is written.</li>
 * </ul>
 *
 * <p>Change the PID with {@link #setAimPID(double, double, double)} /
 * {@link #setRangePID(double, double, double)}, or live from the dashboard (keys
 * "Vision/Aim kP|kI|kD" and "Vision/Range kP|kI|kD").
 */
public class Vision extends SubsystemBase {

  private final String m_name = Constants.Vision.kLimelightName;

  private final PIDController m_aimPid = new PIDController(
      Constants.Vision.kAimP, Constants.Vision.kAimI, Constants.Vision.kAimD);
  private final PIDController m_rangePid = new PIDController(
      Constants.Vision.kRangeP, Constants.Vision.kRangeI, Constants.Vision.kRangeD);

  private double[] m_lastAim = {Constants.Vision.kAimP, Constants.Vision.kAimI, Constants.Vision.kAimD};
  private double[] m_lastRange = {Constants.Vision.kRangeP, Constants.Vision.kRangeI, Constants.Vision.kRangeD};

  public Vision() {
    LimelightHelpers.setPipelineIndex(m_name, Constants.Vision.kAprilTagPipeline);
    LimelightHelpers.setLEDMode_PipelineControl(m_name);
    // Tells MegaTag where the camera sits on the robot (so the pose is the ROBOT's, not the camera's)
    LimelightHelpers.setCameraPose_RobotSpace(m_name,
        Constants.Vision.kCamForwardM, Constants.Vision.kCamSideM, Constants.Vision.kCamUpM,
        Constants.Vision.kCamRollDeg, Constants.Vision.kCamPitchDeg, Constants.Vision.kCamYawDeg);

    m_aimPid.setTolerance(Constants.Vision.kAimToleranceDeg);
    m_rangePid.setTolerance(0.05);

    publishAimGains();
    publishRangeGains();
  }

  // ===================================================================
  //  TARGET DATA
  // ===================================================================

  /** True if the camera sees at least one AprilTag. */
  public boolean hasTarget() { return LimelightHelpers.getTV(m_name); }

  /** ID of the main (best) tag in view, or -1 if none. */
  public int getTagId() { return hasTarget() ? (int) LimelightHelpers.getFiducialID(m_name) : -1; }

  /** Horizontal offset to the tag in degrees (right of the crosshair = +). */
  public double getTx() { return LimelightHelpers.getTX(m_name); }

  /** Vertical offset to the tag in degrees (up = +). */
  public double getTy() { return LimelightHelpers.getTY(m_name); }

  /** Tag area, 0..100 % of the image. */
  public double getTa() { return LimelightHelpers.getTA(m_name); }

  /**
   * Distance from the camera to the tag, in meters, using the camera mount angle and height
   * (requires the tag height / camera pitch / height in Constants.Vision). NaN if no tag.
   */
  public double getDistanceToTagM() {
    if (!hasTarget()) return Double.NaN;
    double angleRad = Math.toRadians(Constants.Vision.kCamPitchDeg + getTy());
    if (Math.abs(Math.tan(angleRad)) < 1e-6) return Double.NaN;
    return (Constants.Vision.kTagHeightM - Constants.Vision.kCamUpM) / Math.tan(angleRad);
  }

  /** Restricts tracking to these tag IDs (e.g. only the ones on your scoring target). */
  public void setTagFilter(int... ids) { LimelightHelpers.SetFiducialIDFiltersOverride(m_name, ids); }

  public void setPipeline(int index) { LimelightHelpers.setPipelineIndex(m_name, index); }

  public void ledsOff() { LimelightHelpers.setLEDMode_ForceOff(m_name); }
  public void ledsOn()  { LimelightHelpers.setLEDMode_PipelineControl(m_name); }

  // ===================================================================
  //  AIM / RANGE PID (for driving toward a tag)
  // ===================================================================

  /**
   * Rotation command (-max..max) that turns the robot to center the tag. Sign follows the WPILib
   * convention: counter-clockwise = +. Returns 0 if there is no tag.
   * Feed it to your swerve's rotation input (flip the sign if your swerve is the opposite).
   */
  public double getAimOutput() {
    if (!hasTarget()) {
      m_aimPid.reset();
      return 0.0;
    }
    // tx > 0 means the tag is to the right -> turn clockwise (negative)
    double out = -m_aimPid.calculate(getTx(), 0.0);
    return MathUtil.clamp(out, -Constants.Vision.kAimMaxOutput, Constants.Vision.kAimMaxOutput);
  }

  /** True when the tag is centered within the tolerance. */
  public boolean isAimed() { return hasTarget() && m_aimPid.atSetpoint(); }

  /**
   * Forward command (-1..1) that approaches / backs off to {@code desiredDistanceM} from the tag.
   * Returns 0 if there is no tag.
   */
  public double getRangeOutput(double desiredDistanceM) {
    double dist = getDistanceToTagM();
    if (Double.isNaN(dist)) {
      m_rangePid.reset();
      return 0.0;
    }
    return MathUtil.clamp(m_rangePid.calculate(dist, desiredDistanceM), -1.0, 1.0);
  }

  /** Changes the aim (rotation) PID gains. Takes effect immediately. */
  public void setAimPID(double kP, double kI, double kD) {
    m_aimPid.setPID(kP, kI, kD);
    m_lastAim = new double[] {kP, kI, kD};
    publishAimGains();
  }

  /** Changes the range (forward) PID gains. Takes effect immediately. */
  public void setRangePID(double kP, double kI, double kD) {
    m_rangePid.setPID(kP, kI, kD);
    m_lastRange = new double[] {kP, kI, kD};
    publishRangeGains();
  }

  public void resetPIDToDefaults() {
    setAimPID(Constants.Vision.kAimP, Constants.Vision.kAimI, Constants.Vision.kAimD);
    setRangePID(Constants.Vision.kRangeP, Constants.Vision.kRangeI, Constants.Vision.kRangeD);
  }

  private void publishAimGains() {
    SmartDashboard.putNumber("Vision/Aim kP", m_lastAim[0]);
    SmartDashboard.putNumber("Vision/Aim kI", m_lastAim[1]);
    SmartDashboard.putNumber("Vision/Aim kD", m_lastAim[2]);
  }

  private void publishRangeGains() {
    SmartDashboard.putNumber("Vision/Range kP", m_lastRange[0]);
    SmartDashboard.putNumber("Vision/Range kI", m_lastRange[1]);
    SmartDashboard.putNumber("Vision/Range kD", m_lastRange[2]);
  }

  // ===================================================================
  //  FIELD POSE (MegaTag2)
  // ===================================================================

  /**
   * Robot field pose from the AprilTags (blue-origin WPILib coordinates), using MegaTag2.
   * Call every loop, passing the gyro yaw and yaw rate. Returns empty when the estimate should be
   * ignored (no tag, too far, or spinning too fast).
   *
   * <p>Later: {@code poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds)}.
   *
   * @param gyroYawDeg robot yaw in degrees (0 = facing the red alliance wall, CCW +)
   * @param gyroYawRateDegPerSec robot yaw rate in degrees/second
   */
  public Optional<PoseEstimate> getPoseEstimate(double gyroYawDeg, double gyroYawRateDegPerSec) {
    LimelightHelpers.SetRobotOrientation(m_name, gyroYawDeg, gyroYawRateDegPerSec, 0, 0, 0, 0);
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

    if (est == null || est.tagCount == 0) return Optional.empty();
    if (Math.abs(gyroYawRateDegPerSec) > Constants.Vision.kMaxYawRateDegPerSec) return Optional.empty();
    if (est.avgTagDist > Constants.Vision.kMaxTagDistanceM) return Optional.empty();
    return Optional.of(est);
  }

  // ===================================================================
  //  PERIODIC
  // ===================================================================
  @Override
  public void periodic() {
    // Live PID tuning from the dashboard
    double[] aim = {
      SmartDashboard.getNumber("Vision/Aim kP", m_lastAim[0]),
      SmartDashboard.getNumber("Vision/Aim kI", m_lastAim[1]),
      SmartDashboard.getNumber("Vision/Aim kD", m_lastAim[2])};
    if (aim[0] != m_lastAim[0] || aim[1] != m_lastAim[1] || aim[2] != m_lastAim[2]) {
      setAimPID(aim[0], aim[1], aim[2]);
    }
    double[] range = {
      SmartDashboard.getNumber("Vision/Range kP", m_lastRange[0]),
      SmartDashboard.getNumber("Vision/Range kI", m_lastRange[1]),
      SmartDashboard.getNumber("Vision/Range kD", m_lastRange[2])};
    if (range[0] != m_lastRange[0] || range[1] != m_lastRange[1] || range[2] != m_lastRange[2]) {
      setRangePID(range[0], range[1], range[2]);
    }

    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putNumber("Vision/TagID", getTagId());
    SmartDashboard.putNumber("Vision/tx", getTx());
    SmartDashboard.putNumber("Vision/Distance (m)", getDistanceToTagM());
  }
}

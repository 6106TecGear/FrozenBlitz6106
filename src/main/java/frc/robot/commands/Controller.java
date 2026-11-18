package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;


public class Controller extends Command {

    public static final double STICK_DEADBAND = 0.08;
    public static final double TRIGGER_THRESHOLD = 0.5;

    /** Extreme 3D Pro needs a bigger deadband (its stick is loose and the twist is noisy). */
    public static final double JOY_DEADBAND = 0.10;
    public static final double JOY_TWIST_DEADBAND = 0.20;

    private final XboxController pad;
    /** Logitech Extreme 3D Pro. null if the 2-port constructor was not used. */
    private final Joystick joy;

    /** Xbox controller only. */
    public Controller(int port) {
        pad = new XboxController(port);
        joy = null;
    }

    /** Xbox controller + Logitech Extreme 3D Pro. */
    public Controller(int xboxPort, int joystickPort) {
        pad = new XboxController(xboxPort);
        joy = new Joystick(joystickPort);
    }

    public XboxController raw() { return pad; }

    /** The raw Extreme 3D joystick (null if there is none). */
    public Joystick rawJoystick() { return joy; }

    // =====================================================================
    //  FACE BUTTONS: A, B, X, Y
    // =====================================================================
    public boolean aHeld()     { return pad.getAButton(); }
    public boolean aPressed()  { return pad.getAButtonPressed(); }
    public boolean aReleased() { return pad.getAButtonReleased(); }

    public boolean bHeld()     { return pad.getBButton(); }
    public boolean bPressed()  { return pad.getBButtonPressed(); }
    public boolean bReleased() { return pad.getBButtonReleased(); }

    public boolean xHeld()     { return pad.getXButton(); }
    public boolean xPressed()  { return pad.getXButtonPressed(); }
    public boolean xReleased() { return pad.getXButtonReleased(); }

    public boolean yHeld()     { return pad.getYButton(); }
    public boolean yPressed()  { return pad.getYButtonPressed(); }
    public boolean yReleased() { return pad.getYButtonReleased(); }

    // =====================================================================
    //  BUMPERS: LB (L1) and RB (R1)
    // =====================================================================
    public boolean lbHeld()     { return pad.getLeftBumperButton(); }
    public boolean lbPressed()  { return pad.getLeftBumperButtonPressed(); }
    public boolean lbReleased() { return pad.getLeftBumperButtonReleased(); }

    public boolean rbHeld()     { return pad.getRightBumperButton(); }
    public boolean rbPressed()  { return pad.getRightBumperButtonPressed(); }
    public boolean rbReleased() { return pad.getRightBumperButtonReleased(); }

    // =====================================================================
    //  CENTER BUTTONS: Back and Start
    // =====================================================================
    public boolean backHeld()     { return pad.getBackButton(); }
    public boolean backPressed()  { return pad.getBackButtonPressed(); }
    public boolean backReleased() { return pad.getBackButtonReleased(); }

    public boolean startHeld()     { return pad.getStartButton(); }
    public boolean startPressed()  { return pad.getStartButtonPressed(); }
    public boolean startReleased() { return pad.getStartButtonReleased(); }

    // =====================================================================
    //  STICK CLICKS: press the stick down (L3 / R3)
    // =====================================================================
    public boolean leftStickClickHeld()     { return pad.getLeftStickButton(); }
    public boolean leftStickClickPressed()  { return pad.getLeftStickButtonPressed(); }
    public boolean leftStickClickReleased() { return pad.getLeftStickButtonReleased(); }

    public boolean rightStickClickHeld()     { return pad.getRightStickButton(); }
    public boolean rightStickClickPressed()  { return pad.getRightStickButtonPressed(); }
    public boolean rightStickClickReleased() { return pad.getRightStickButtonReleased(); }

    // =====================================================================
    //  STICKS (analog, -1 to +1, with deadband)
    //  X: left = -1, right = +1.   Y: up = +1, down = -1 (we flip WPILib's sign).
    // =====================================================================
    public double leftX()  { return deadband(pad.getLeftX()); }
    public double leftY()  { return deadband(-pad.getLeftY()); }
    public double rightX() { return deadband(pad.getRightX()); }
    public double rightY() { return deadband(-pad.getRightY()); }

    /** Same as above but WITHOUT deadband. */
    public double leftXRaw()  { return pad.getLeftX(); }
    public double leftYRaw()  { return -pad.getLeftY(); }
    public double rightXRaw() { return pad.getRightX(); }
    public double rightYRaw() { return -pad.getRightY(); }

    // =====================================================================
    //  TRIGGERS: LT (L2) and RT (R2), 0 (released) to 1 (fully pressed)
    // =====================================================================
    public double lt() { return pad.getLeftTriggerAxis(); }
    public double rt() { return pad.getRightTriggerAxis(); }

    /** Right minus left: -1 to +1. Handy for "forward/backward" with the triggers. */
    public double triggersDifference() { return rt() - lt(); }

    /** Triggers used like buttons. */
    public boolean ltHeld() { return lt() > TRIGGER_THRESHOLD; }
    public boolean rtHeld() { return rt() > TRIGGER_THRESHOLD; }

    private boolean ltWasHeld = false;
    private boolean rtWasHeld = false;

    /** True ONE time when LT crosses the threshold. Call it once per loop. */
    public boolean ltPressed() {
        boolean now = ltHeld();
        boolean result = now && !ltWasHeld;
        ltWasHeld = now;
        return result;
    }

    /** True ONE time when RT crosses the threshold. Call it once per loop. */
    public boolean rtPressed() {
        boolean now = rtHeld();
        boolean result = now && !rtWasHeld;
        rtWasHeld = now;
        return result;
    }

    // =====================================================================
    //  D-PAD (POV hat): angle in degrees, -1 = not pressed
    //  0 = up, 90 = right, 180 = down, 270 = left
    // =====================================================================
    public int dpadAngle() { return pad.getPOV(); }

    public boolean dpadUp()        { return pad.getPOV() == 0; }
    public boolean dpadUpRight()   { return pad.getPOV() == 45; }
    public boolean dpadRight()     { return pad.getPOV() == 90; }
    public boolean dpadDownRight() { return pad.getPOV() == 135; }
    public boolean dpadDown()      { return pad.getPOV() == 180; }
    public boolean dpadDownLeft()  { return pad.getPOV() == 225; }
    public boolean dpadLeft()      { return pad.getPOV() == 270; }
    public boolean dpadUpLeft()    { return pad.getPOV() == 315; }
    public boolean dpadNone()      { return pad.getPOV() == -1; }

    // =====================================================================
    //  RUMBLE (vibration), value 0 to 1
    // =====================================================================
    public void rumbleLeft(double strength)  { pad.setRumble(RumbleType.kLeftRumble, strength); }
    public void rumbleRight(double strength) { pad.setRumble(RumbleType.kRightRumble, strength); }
    public void rumbleBoth(double strength)  { pad.setRumble(RumbleType.kBothRumble, strength); }
    public void rumbleOff()                  { pad.setRumble(RumbleType.kBothRumble, 0.0); }

    // =====================================================================
    //  EXTRAS
    // =====================================================================
    /** True if the controller is plugged in and the Driver Station sees it. */
    public boolean isConnected() { return pad.isConnected(); }

    /** True if ANY face button, bumper, or center button is held. */
    public boolean anyButtonHeld() {
        return aHeld() || bHeld() || xHeld() || yHeld()
            || lbHeld() || rbHeld() || backHeld() || startHeld()
            || leftStickClickHeld() || rightStickClickHeld();
    }

    // =====================================================================
    //  LOGITECH EXTREME 3D PRO (flight joystick)
    //
    //  Axes:    X (left/right), Y (forward/back), Twist (rotate handle), Slider (base throttle)
    //  Buttons: 1 = trigger, 2 = thumb (side), 3-6 = top of the handle,
    //           7-12 = base (7,8 top row / 9,10 middle / 11,12 bottom)
    //  Hat:     the 8-way POV on top of the handle
    //
    //  Same style as the Xbox part: right = +X, forward (push away) = +Y.
    //  Every method is safe to call if no joystick was given: it returns 0 / false.
    // =====================================================================

    // WPILib Joystick axis indexes for the Extreme 3D Pro
    private static final int JOY_AXIS_X = 0;
    private static final int JOY_AXIS_Y = 1;
    private static final int JOY_AXIS_TWIST = 2;
    private static final int JOY_AXIS_SLIDER = 3;

    /** True if the joystick exists and the Driver Station sees it. */
    public boolean joyIsConnected() { return joy != null && joy.isConnected(); }

    // ---- Axes (with deadband) ----
    public double joyX()     { return joy == null ? 0.0 : deadband(joy.getRawAxis(JOY_AXIS_X), JOY_DEADBAND); }
    /** Forward (push away from you) = +1. */
    public double joyY()     { return joy == null ? 0.0 : deadband(-joy.getRawAxis(JOY_AXIS_Y), JOY_DEADBAND); }
    /** Twist clockwise = +1. */
    public double joyTwist() { return joy == null ? 0.0 : deadband(joy.getRawAxis(JOY_AXIS_TWIST), JOY_TWIST_DEADBAND); }

    // ---- Axes (no deadband) ----
    public double joyXRaw()     { return joy == null ? 0.0 : joy.getRawAxis(JOY_AXIS_X); }
    public double joyYRaw()     { return joy == null ? 0.0 : -joy.getRawAxis(JOY_AXIS_Y); }
    public double joyTwistRaw() { return joy == null ? 0.0 : joy.getRawAxis(JOY_AXIS_TWIST); }

    /** Base slider as 0..1: slider pushed all the way FORWARD (top) = 1, all the way back = 0. */
    public double joySlider() {
        if (joy == null) return 0.0;
        return (1.0 - joy.getRawAxis(JOY_AXIS_SLIDER)) / 2.0;
    }

    /** Base slider as -1..+1 (forward = +1). */
    public double joySliderSigned() { return joy == null ? 0.0 : -joy.getRawAxis(JOY_AXIS_SLIDER); }

    // ---- Buttons ----
    /** Any button 1..12 (1 = trigger, 2 = thumb). */
    public boolean joyButtonHeld(int button)     { return joy != null && joy.getRawButton(button); }
    public boolean joyButtonPressed(int button)  { return joy != null && joy.getRawButtonPressed(button); }
    public boolean joyButtonReleased(int button) { return joy != null && joy.getRawButtonReleased(button); }

    public boolean joyTriggerHeld()     { return joyButtonHeld(1); }
    public boolean joyTriggerPressed()  { return joyButtonPressed(1); }
    public boolean joyTriggerReleased() { return joyButtonReleased(1); }

    public boolean joyThumbHeld()     { return joyButtonHeld(2); }
    public boolean joyThumbPressed()  { return joyButtonPressed(2); }
    public boolean joyThumbReleased() { return joyButtonReleased(2); }

    // ---- Hat (POV): same angles as the Xbox D-pad ----
    public int joyHatAngle() { return joy == null ? -1 : joy.getPOV(); }

    public boolean joyHatUp()    { return joyHatAngle() == 0; }
    public boolean joyHatRight() { return joyHatAngle() == 90; }
    public boolean joyHatDown()  { return joyHatAngle() == 180; }
    public boolean joyHatLeft()  { return joyHatAngle() == 270; }
    public boolean joyHatNone()  { return joyHatAngle() == -1; }

    /** True if any of the 12 buttons is held. */
    public boolean joyAnyButtonHeld() {
        if (joy == null) return false;
        for (int b = 1; b <= 12; b++) {
            if (joy.getRawButton(b)) return true;
        }
        return false;
    }

    private static double deadband(double value, double zone) {
        if (Math.abs(value) < zone) return 0.0;
        return Math.signum(value) * (Math.abs(value) - zone) / (1.0 - zone);
    }

    private static double deadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) return 0.0;
        return Math.signum(value) * (Math.abs(value) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
    }
}
package frc.robot.commands;

// ===================== CTRE Phoenix 6 (Kraken X60 / Falcon 500 = TalonFX) =====================
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// ===================== CTRE Phoenix 5 (Talon SRX + CIM / Mini CIM / 775pro) =====================
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// ===================== REV (REVLib 2025+: SPARK MAX + NEO / NEO 550 / brushed CIM) =====================
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * MotorPID: a library of functions for controlling motors with PID.
 *
 * Every motor uses the SAME functions and the SAME units:
 *   - position   -> output-shaft rotations
 *   - velocity   -> output-shaft RPM
 *   - percent    -> -1.0 to 1.0
 *
 * Factories (create a ready-to-use motor):
 *   MotorPID.kraken(id)                   Kraken X60 / Falcon 500 (TalonFX, Phoenix 6)
 *   MotorPID.neo(id)                      NEO on a SPARK MAX
 *   MotorPID.neo550(id)                   NEO 550 on a SPARK MAX (low default current limit)
 *   MotorPID.cimSparkMax(id, ticksPerRev) Brushed CIM / Mini CIM on a SPARK MAX (needs an encoder for PID)
 *   MotorPID.talonSRX(id, ticksPerRev)    Talon SRX + CIM/775 with an encoder (e.g. 4096 ticks per rev)
 *
 * Common functions (Motor interface):
 *   setPercent, setVelocityRPM, setPosition, stop,
 *   getPosition, getVelocityRPM, resetPosition,
 *   setVelocityPID, setPositionPID, setInverted, setBrake, setCurrentLimit
 */
public class MotorPID extends Command {

    // =====================================================================
    //  COMMON INTERFACE
    // =====================================================================
    public interface Motor {
        /** Direct output -1..1 (no PID). */
        void setPercent(double percent);

        /** Target velocity in RPM (velocity PID). */
        void setVelocityRPM(double rpm);

        /** Target position in rotations (position PID). */
        void setPosition(double rotations);

        void stop();

        double getPosition();       // rotations
        double getVelocityRPM();    // RPM

        /** Sets the current position to "value" (normally 0). */
        void resetPosition(double value);

        /** Velocity gains (slot 0). kS/kV are in volts on Talon/Kraken; kV is feedforward on REV. */
        void setVelocityPID(double kP, double kI, double kD, double kS, double kV);

        /** Position gains (slot 1). */
        void setPositionPID(double kP, double kI, double kD);

        void setInverted(boolean inverted);
        void setBrake(boolean brake);
        void setCurrentLimit(double amps);
    }

    // =====================================================================
    //  FACTORIES
    // =====================================================================
    public static Motor kraken(int id) { return new KrakenMotor(id); }
    public static Motor neo(int id) { return new SparkMotor(id, MotorType.kBrushless, 40, 0); }
    public static Motor neo550(int id) { return new SparkMotor(id, MotorType.kBrushless, 20, 0); }
    /** Brushed CIM on a SPARK MAX. PID needs an encoder wired to the SPARK MAX; ticksPerRev = encoder CPR. */
    public static Motor cimSparkMax(int id, int ticksPerRev) { return new SparkMotor(id, MotorType.kBrushed, 40, ticksPerRev); }
    public static Motor talonSRX(int id, int ticksPerRev) { return new TalonSRXMotor(id, ticksPerRev); }

    // =====================================================================
    //  KRAKEN X60 / FALCON 500  (TalonFX, Phoenix 6)
    // =====================================================================
    private static class KrakenMotor implements Motor {
        private final TalonFX talon;
        private final TalonFXConfiguration config = new TalonFXConfiguration();
        private final VelocityVoltage velReq = new VelocityVoltage(0).withSlot(0);
        private final PositionVoltage posReq = new PositionVoltage(0).withSlot(1);
        private final DutyCycleOut dutyReq = new DutyCycleOut(0);

        KrakenMotor(int id) {
            talon = new TalonFX(id);
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            // Starting values (same as your original code)
            config.Slot0.kS = 0.1;  config.Slot0.kV = 0.12; config.Slot0.kP = 0.40;
            config.Slot1.kP = 2.4;  config.Slot1.kD = 0.1;
            apply();
            talon.setPosition(0.0);
        }

        private void apply() { talon.getConfigurator().apply(config); }

        @Override public void setPercent(double p) { talon.setControl(dutyReq.withOutput(p)); }
        @Override public void setVelocityRPM(double rpm) { talon.setControl(velReq.withVelocity(rpm / 60.0)); }
        @Override public void setPosition(double rot) { talon.setControl(posReq.withPosition(rot)); }
        @Override public void stop() { talon.stopMotor(); }
        @Override public double getPosition() { return talon.getPosition().getValueAsDouble(); }
        @Override public double getVelocityRPM() { return talon.getVelocity().getValueAsDouble() * 60.0; }
        @Override public void resetPosition(double v) { talon.setPosition(v); }

        @Override public void setVelocityPID(double kP, double kI, double kD, double kS, double kV) {
            config.Slot0.kP = kP; config.Slot0.kI = kI; config.Slot0.kD = kD;
            config.Slot0.kS = kS; config.Slot0.kV = kV;
            apply();
        }

        @Override public void setPositionPID(double kP, double kI, double kD) {
            config.Slot1.kP = kP; config.Slot1.kI = kI; config.Slot1.kD = kD;
            apply();
        }

        @Override public void setInverted(boolean inv) {
            config.MotorOutput.Inverted = inv ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            apply();
        }

        @Override public void setBrake(boolean brake) {
            config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            apply();
        }

        @Override public void setCurrentLimit(double a) {
            config.CurrentLimits.SupplyCurrentLimit = a;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            apply();
        }
    }

    // =====================================================================
    //  NEO / NEO 550 / brushed CIM 
    // =====================================================================
    private static class SparkMotor implements Motor {
        private final SparkMax spark;
        private final SparkMaxConfig config = new SparkMaxConfig();
        private final SparkClosedLoopController pid;
        private final RelativeEncoder encoder;

        SparkMotor(int id, MotorType type, int currentLimit, int ticksPerRev) {
            spark = new SparkMax(id, type);
            pid = spark.getClosedLoopController();
            encoder = spark.getEncoder();

            config.idleMode(IdleMode.kBrake);
            config.smartCurrentLimit(currentLimit);
            // Position in rotations and velocity in RPM (default on NEO/NEO 550).
            // For a brushed CIM with an external encoder the conversion factor must be set.
            if (type == MotorType.kBrushed && ticksPerRev > 0) {
                config.encoder.positionConversionFactor(1.0 / ticksPerRev);
                config.encoder.velocityConversionFactor(1.0 / ticksPerRev);
            }
            // Slot 0 = velocity, Slot 1 = position
            config.closedLoop.pid(0.0001, 0, 0, ClosedLoopSlot.kSlot0);
            config.closedLoop.velocityFF(1.0 / 5676.0, ClosedLoopSlot.kSlot0); // NEO free speed: ~5676 RPM
            config.closedLoop.pid(0.1, 0, 0, ClosedLoopSlot.kSlot1);
            apply();
        }
        

        private void apply() {
            spark.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        }

        @Override public void setPercent(double p) { spark.set(p); }
        @Override public void setVelocityRPM(double rpm) { pid.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0); }
        @Override public void setPosition(double rot) { pid.setSetpoint(rot, ControlType.kPosition, ClosedLoopSlot.kSlot1); }
        @Override public void stop() { spark.stopMotor(); }
        @Override public double getPosition() { return encoder.getPosition(); }
        @Override public double getVelocityRPM() { return encoder.getVelocity(); }
        @Override public void resetPosition(double v) { encoder.setPosition(v); }

        @Override public void setVelocityPID(double kP, double kI, double kD, double kS, double kV) {
            // REV has no kS; kV is used as the velocity feedforward
            config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
            config.closedLoop.velocityFF(kV, ClosedLoopSlot.kSlot0);
            apply();
        }

        @Override public void setPositionPID(double kP, double kI, double kD) {
            config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot1);
            apply();
        }

        @Override public void setInverted(boolean inv) { config.inverted(inv); apply(); }
        @Override public void setBrake(boolean brake) { config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast); apply(); }
        @Override public void setCurrentLimit(double a) { config.smartCurrentLimit((int) a); apply(); }
    }

    // =====================================================================
    //  TALON SRX + CIM / Mini CIM / 775pro  (Phoenix 5)
    // =====================================================================
    private static class TalonSRXMotor implements Motor {
        private final TalonSRX talon;
        private final double ticksPerRev;

        TalonSRXMotor(int id, int ticksPerRev) {
            this.ticksPerRev = ticksPerRev;
            talon = new TalonSRX(id);
            talon.configFactoryDefault();
            talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configContinuousCurrentLimit(30);
            talon.enableCurrentLimit(true);
            // Slot 0 = velocity, Slot 1 = position
            talon.config_kP(0, 0.1); talon.config_kF(0, 0.05);
            talon.config_kP(1, 0.5);
            talon.setSelectedSensorPosition(0);
        }

        // Phoenix 5 measures velocity in ticks per 100 ms
        private double rpmToTicksPer100ms(double rpm) { return rpm / 600.0 * ticksPerRev; }

        @Override public void setPercent(double p) { talon.set(ControlMode.PercentOutput, p); }
        @Override public void setVelocityRPM(double rpm) { talon.selectProfileSlot(0, 0); talon.set(ControlMode.Velocity, rpmToTicksPer100ms(rpm)); }
        @Override public void setPosition(double rot) { talon.selectProfileSlot(1, 0); talon.set(ControlMode.Position, rot * ticksPerRev); }
        @Override public void stop() { talon.set(ControlMode.PercentOutput, 0); }
        @Override public double getPosition() { return talon.getSelectedSensorPosition() / ticksPerRev; }
        @Override public double getVelocityRPM() { return talon.getSelectedSensorVelocity() * 600.0 / ticksPerRev; }
        @Override public void resetPosition(double v) { talon.setSelectedSensorPosition(v * ticksPerRev); }

        @Override public void setVelocityPID(double kP, double kI, double kD, double kS, double kV) {
            // Phoenix 5 uses kF instead of kV; there is no kS.
            talon.config_kP(0, kP); talon.config_kI(0, kI); talon.config_kD(0, kD); talon.config_kF(0, kV);
        }

        @Override public void setPositionPID(double kP, double kI, double kD) {
            talon.config_kP(1, kP); talon.config_kI(1, kI); talon.config_kD(1, kD);
        }

        @Override public void setInverted(boolean inv) { talon.setInverted(inv); }
        @Override public void setBrake(boolean brake) { talon.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast); }
        @Override public void setCurrentLimit(double a) { talon.configContinuousCurrentLimit((int) a); talon.enableCurrentLimit(true); }
    }

    // =====================================================================
    //  STATIC UTILITIES (work with any motor)
    // =====================================================================

    /** Applies a deadband and rescales so output starts at 0 right after leaving the deadband. */
    public static double deadband(double value, double zone) {
        if (Math.abs(value) < zone) return 0.0;
        return Math.signum(value) * (Math.abs(value) - zone) / (1.0 - zone);
    }

    /** Clamps a value between min and max. */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Rounds to fixed steps, e.g. roundToStep(x, 12.5). */
    public static double roundToStep(double value, double step) {
        return step * Math.round(value / step);
    }

    /** True if the position is within tolerance of the target. */
    public static boolean atPosition(Motor m, double targetRot, double tolRot) {
        return Math.abs(m.getPosition() - targetRot) <= tolRot;
    }

    /** True if the velocity is within tolerance of the target. */
    public static boolean atVelocity(Motor m, double targetRPM, double tolRPM) {
        return Math.abs(m.getVelocityRPM() - targetRPM) <= tolRPM;
    }

    /** Makes "follower" repeat whatever "leader" does. Use the returned wrapper instead of the original motors. */
    public static Motor follow(Motor leader, Motor follower, boolean opposite) {
        final double sign = opposite ? -1.0 : 1.0;
        return new Motor() {
            @Override public void setPercent(double p) { leader.setPercent(p); follower.setPercent(sign * p); }
            @Override public void setVelocityRPM(double r) { leader.setVelocityRPM(r); follower.setVelocityRPM(sign * r); }
            @Override public void setPosition(double r) { leader.setPosition(r); follower.setPosition(sign * r); }
            @Override public void stop() { leader.stop(); follower.stop(); }
            @Override public double getPosition() { return leader.getPosition(); }
            @Override public double getVelocityRPM() { return leader.getVelocityRPM(); }
            @Override public void resetPosition(double v) { leader.resetPosition(v); follower.resetPosition(sign * v); }
            @Override public void setVelocityPID(double p, double i, double d, double s, double v) { leader.setVelocityPID(p, i, d, s, v); follower.setVelocityPID(p, i, d, s, v); }
            @Override public void setPositionPID(double p, double i, double d) { leader.setPositionPID(p, i, d); follower.setPositionPID(p, i, d); }
            @Override public void setInverted(boolean inv) { leader.setInverted(inv); }
            @Override public void setBrake(boolean b) { leader.setBrake(b); follower.setBrake(b); }
            @Override public void setCurrentLimit(double a) { leader.setCurrentLimit(a); follower.setCurrentLimit(a); }
        };
    }

    /** Publishes a motor's position and velocity to SmartDashboard. */
    public static void publish(String name, Motor m) {
        SmartDashboard.putNumber(name + " Position (rot)", m.getPosition());
        SmartDashboard.putNumber(name + " Velocity (RPM)", m.getVelocityRPM());
    }

}
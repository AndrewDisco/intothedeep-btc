package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class HorizontalSlider extends SubsystemBase {

    private MotorEx sliderMotor;
    private PIDFController pidfController;
    private FtcDashboard dashboard;

    // === PIDF coefficients ===
    public static double kP = 0.001;   // Proportional gain
    public static double kI = 0.0;     // Integral gain
    public static double kD = 0.000015;  // Derivative gain
    public static double kF = 0.0;     // Feed-forward gain

    // Other tunable parameters
    public static double tolerance = 200.0;  // Position tolerance in ticks
    public static double MIN_POSITION = 0;
    public static double MAX_POSITION = 35000;

    // Target position
    private double targetPosition = 0;

    // Previous PIDF values to detect changes
    private double prevKP = kP;
    private double prevKI = kI;
    private double prevKD = kD;
    private double prevKF = kF;
    private double prevTolerance = tolerance;

    public HorizontalSlider(HardwareMap hardwareMap) {
        // Initialize motor
        sliderMotor = new MotorEx(hardwareMap, "glisiere");

        // Configure motor settings - INVERTED to fix backwards movement
        sliderMotor.setInverted(true);
        sliderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        sliderMotor.setRunMode(Motor.RunMode.RawPower);

        // Reset encoder
        sliderMotor.resetEncoder();

        // Initialize PIDF controller with dashboard-tunable values
        pidfController = new PIDFController(kP, kI, kD, kF);
        pidfController.setTolerance(tolerance);

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();

        // Set initial target to current position
        targetPosition = getCurrentPosition();
        pidfController.setSetPoint(targetPosition);
    }

    @Override
    public void periodic() {
        // Check for PIDF coefficient changes from dashboard
        updatePIDFFromDashboard();

        // Update PIDF controller with current position
        double currentPosition = getCurrentPosition();
        double output = pidfController.calculate(currentPosition);

        // Apply power to motor
        sliderMotor.set(output);

        // Send telemetry to dashboard
        sendDashboardTelemetry();
    }

    private void updatePIDFFromDashboard() {
        // Check if any PIDF values have changed
        if (kP != prevKP || kI != prevKI || kD != prevKD || kF != prevKF) {
            pidfController.setPIDF(kP, kI, kD, kF);
            prevKP = kP;
            prevKI = kI;
            prevKD = kD;
            prevKF = kF;
        }

        // Check if tolerance has changed
        if (tolerance != prevTolerance) {
            pidfController.setTolerance(tolerance);
            prevTolerance = tolerance;
        }
    }

    private void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();

        // Slider status
        packet.put("Slider/Current Position", getCurrentPosition());
        packet.put("Slider/Target Position", getTargetPosition());
        packet.put("Slider/Position Error", getPositionError());
        packet.put("Slider/Motor Power", getMotorPower());
        packet.put("Slider/At Target", atTargetPosition());

        // PIDF values (for monitoring)
        packet.put("Slider/PID/kP", kP);
        packet.put("Slider/PID/kI", kI);
        packet.put("Slider/PID/kD", kD);
        packet.put("Slider/PID/kF", kF);
        packet.put("Slider/PID/Tolerance", tolerance);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Get current position of the slider in encoder ticks
     */
    public double getCurrentPosition() {
        return sliderMotor.getCurrentPosition();
    }

    /**
     * Set target position for the slider
     * @param position Target position in encoder ticks
     */
    public void setTargetPosition(double position) {
        // Clamp position within limits
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        pidfController.setSetPoint(targetPosition);
    }

    /**
     * Get current target position
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Move slider by a relative amount
     * @param deltaPosition Amount to move (positive = extend, negative = retract)
     */
    public void moveRelative(double deltaPosition) {
        setTargetPosition(targetPosition + deltaPosition);
    }

    /**
     * Check if slider is at target position (within tolerance)
     */
    public boolean atTargetPosition() {
        return pidfController.atSetPoint();
    }

    /**
     * Get position error (target - current)
     */
    public double getPositionError() {
        return targetPosition - getCurrentPosition();
    }

    /**
     * Set slider to fully retracted position
     */
    public void retract() {
        setTargetPosition(MIN_POSITION);
    }

    /**
     * Set slider to fully extended position
     */
    public void extend() {
        setTargetPosition(MAX_POSITION);
    }

    /**
     * Set manual power to slider motor (bypasses PID control)
     * Use for testing or manual override
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setManualPower(double power) {
        sliderMotor.set(power);
    }

    /**
     * Stop the slider motor
     */
    public void stop() {
        sliderMotor.set(0);
    }

    /**
     * Reset the encoder position to zero
     */
    public void resetEncoder() {
        sliderMotor.resetEncoder();
        targetPosition = 0;
        pidfController.setSetPoint(0);
    }

    /**
     * Update PIDF coefficients (useful for tuning)
     */
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        pidfController.setPIDF(p, i, d, f);
    }

    /**
     * Set position tolerance
     */
    public void setTolerance(double tol) {
        tolerance = tol;
        pidfController.setTolerance(tol);
    }

    /**
     * Get current motor power
     */
    public double getMotorPower() {
        return sliderMotor.get();
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class VerticalExtension extends SubsystemBase {

    private PIDFController pidfController;
    private FtcDashboard dashboard;

    // === PIDF coefficients ===
    public static double kP = 0.0007;   // Proportional gain
    public static double kI = 0d;     // Integral gain
    public static double kD = 0.00001;  // Derivative gain
    public static double kF = 0.02;     // Feed-forward gain

    // Other tunable parameters
    public static double tolerance = 500.0;  // Position tolerance in ticks
    public static double MIN_POSITION = 0;
    public static double MAX_POSITION = 35000;
    public static double INTAKE_POSITION = 0;
    public static double OUTTAKE_POSITION = 28000;

    // Target position
    private double targetPosition = 0;

    // Previous PIDF values to detect changes
    private double prevKP = kP;
    private double prevKI = kI;
    private double prevKD = kD;
    private double prevKF = kF;
    private double prevTolerance = tolerance;

    Motor m1, m2;
    Motor.Encoder encoder;
    public VerticalExtension(HardwareMap hardwareMap) {
        m1 = new Motor(hardwareMap, "v1");
        m2 = new Motor(hardwareMap, "v2");

        m1.setInverted(true);
        m2.setInverted(false);

        Motor encoderMotor = new Motor(hardwareMap, "enc");
        encoder = encoderMotor.encoder;
        encoder.setDirection(Motor.Direction.REVERSE);
        encoder.reset();

        pidfController = new PIDFController(kP, kI, kD, 0d);
        pidfController.setTolerance(tolerance);

        dashboard = FtcDashboard.getInstance();

        targetPosition = getCurrentPosition();
        pidfController.setSetPoint(targetPosition);
    }

    public void set(double power) {
//        power = Math.max(power, -0.5);
        m1.set(power);
        m2.set(power);
    }



    @Override
    public void periodic() {
        // Check for PIDF coefficient changes from dashboard
        updatePIDFFromDashboard();

        // Update PIDF controller with current position
        double currentPosition = getCurrentPosition();
        double output = pidfController.calculate(currentPosition) + kF;

        // Apply power to motor
        this.set(output);

        // Send telemetry to dashboard
        sendDashboardTelemetry();
    }

    private void updatePIDFFromDashboard() {
        // Check if any PIDF values have changed
        if (kP != prevKP || kI != prevKI || kD != prevKD || kF != prevKF) {
            pidfController.setPIDF(kP, kI, kD, 0d);
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
            return encoder.getPosition();
        }

        /**
         * Set target position for the slider
         * @param position Target position in encoder ticks
         */
        public void setTargetPosition(double position) {
            // Clamp position within limits
            position = Math.max(MIN_POSITION, position);
            position = Math.min(MAX_POSITION, position);
            targetPosition = position;
            pidfController.setSetPoint(targetPosition);
        }

        /**
         * Get current target position
         */
        public double getTargetPosition() {
            return targetPosition;
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
            this.set(power);
        }

        /**
         * Stop the slider motor
         */
        public void stop() {
            this.set(0);
        }

        /**
         * Reset the encoder position to zero
         */
        public void resetEncoder() {
            encoder.reset();
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
            pidfController.setPIDF(p, i, d, 0d);
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
            return m1.get();
        }


}

package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Pickup Test", group = "Test")
public class PickupTest extends LinearOpMode {
    Limelight3A limelight;
    Servo baseRotation, updown, gripperRotation, gripper;
    double imageWidth, imageHeight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configure bulk caching for better performance
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Set up telemetry with dashboard support
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Reset command scheduler
        CommandScheduler.getInstance().reset();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight == null) {
            telemetry.addData("Error", "Limelight not found in hardware map");
        } else {
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
            LLStatus status = limelight.getStatus();
            imageWidth = 640;
            imageHeight = 480;
            telemetry.addData("Limelight", "Initialized - Width: %.0f, Height: %.0f", imageWidth, imageHeight);
        }

        // Initialize Servos with error checking
        baseRotation = hardwareMap.get(Servo.class, "armRotation");
        updown = hardwareMap.get(Servo.class, "armPivot");
        gripperRotation = hardwareMap.get(Servo.class, "roll");
        gripper = hardwareMap.get(Servo.class, "gripper");

        if (baseRotation == null || updown == null || gripperRotation == null || gripper == null) {
            telemetry.addData("Error", "One or more servos not found in hardware map");
        } else {
            telemetry.addData("Servos", "Initialized successfully");
            // Set initial positions
            baseRotation.setPosition(0.5);  // Center
            updown.setPosition(0.5);        // Middle
            gripperRotation.setPosition(0.5); // 90 degrees
            gripper.setPosition(0.0);       // Open
        }

        waitForStart();

        while (opModeIsActive()) {
            // Run the command scheduler
            CommandScheduler.getInstance().run();

            LLResult result = limelight.getLatestResult();
            telemetry.addData("Status", "Processing...");

            if (result != null && result.isValid()) {
                double[] pythonOutputs = result.getPythonOutput();
                telemetry.addData("Limelight", "Result valid, Pipeline: %d", result.getPipelineIndex());

                if (pythonOutputs != null && pythonOutputs.length >= 9) {
                    // Parse Python outputs
                    int blueDetected = (int) pythonOutputs[0];
                    double blueX = pythonOutputs[1];
                    double blueY = pythonOutputs[2];
                    double blueAngle = pythonOutputs[3];
                    int redDetected = (int) pythonOutputs[4];
                    double redX = pythonOutputs[5];
                    double redY = pythonOutputs[6];
                    double redAngle = pythonOutputs[7];
                    double selectedAngle = pythonOutputs[8];

                    // Debug telemetry for raw data
                    telemetry.addData("Blue", "Det: %d, X: %.1f, Y: %.1f, Angle: %.1f", blueDetected, blueX, blueY, blueAngle);
                    telemetry.addData("Red", "Det: %d, X: %.1f, Y: %.1f, Angle: %.1f", redDetected, redX, redY, redAngle);
                    telemetry.addData("Selected Angle", "%.1f", selectedAngle);

                    // Select target (closest to bottom)
                    double targetX, targetY, targetAngle;
                    if (blueDetected == 1 && redDetected == 1) {
                        if (blueY > redY) {
                            targetX = blueX;
                            targetY = blueY;
                            targetAngle = blueAngle;
                        } else {
                            targetX = redX;
                            targetY = redY;
                            targetAngle = redAngle;
                        }
                    } else if (blueDetected == 1) {
                        targetX = blueX;
                        targetY = blueY;
                        targetAngle = blueAngle;
                    } else if (redDetected == 1) {
                        targetX = redX;
                        targetY = redY;
                        targetAngle = redAngle;
                    } else {
                        telemetry.addData("Status", "No objects detected");
                        telemetry.update();
                        continue;
                    }

                    telemetry.addData("Target", "X: %.1f, Y: %.1f, Angle: %.1f", targetX, targetY, targetAngle);

                    // Align the arm
                    alignArm(targetX, targetY, targetAngle);

                    // Manual gripper control with gamepad
                    if (gamepad1.a) {
                        gripper.setPosition(1.0); // Close
                        telemetry.addData("Gripper", "Closed (1.0)");
                    } else {
                        gripper.setPosition(0.0); // Open
                        telemetry.addData("Gripper", "Open (0.0)");
                    }
                } else {
                    telemetry.addData("Python Outputs", "Invalid or insufficient length: %d",
                            pythonOutputs != null ? pythonOutputs.length : -1);
                }
            } else {
                telemetry.addData("Limelight Result", "Null or invalid");
            }
            telemetry.update();
        }
    }

    private void alignArm(double targetX, double targetY, double targetAngle) {
        // Calculate center and offsets
        double centerX = imageWidth / 2.0;
        double centerY = imageHeight / 2.0;
        double offsetX = targetX - centerX; // Negative = left, Positive = right
        double offsetY = targetY - centerY; // Negative = up, Positive = down

        // Servo scaling factors (adjust these as needed)
        double baseScale = 0.002; // Increased for more noticeable movement

        // Current servo positions
        double basePos = baseRotation.getPosition();
        double updownPos = updown.getPosition();
        double gripperRotPos = gripperRotation.getPosition();

        // Adjust baseRotation (0 = left, 1 = right)
        double baseAdjustment = offsetX * baseScale;
        double newBasePos = Math.max(0, Math.min(1, basePos + baseAdjustment));
        baseRotation.setPosition(newBasePos);

        // Adjust updown (0 = up, 1 = down)
        double targetUpdown = targetY / imageHeight; // Maps 0 to imageHeight -> 0 to 1
        updown.setPosition(targetUpdown);

        // Adjust gripperRotation (assuming 0-180 degrees)
        // Normalize angle (assuming Python script gives -45 to 45 degrees)
        double normalizedAngle = (targetAngle + 45) / 90.0; // Maps -45 to 45 -> 0 to 1
        double gripperAngle = Math.max(0, Math.min(1, normalizedAngle));
        gripperRotation.setPosition(gripperAngle);

        // Extensive debug telemetry
        telemetry.addData("Image", "W: %.0f, H: %.0f, CenterX: %.1f, CenterY: %.1f",
                imageWidth, imageHeight, centerX, centerY);
        telemetry.addData("Offsets", "X: %.1f, Y: %.1f", offsetX, offsetY);
        telemetry.addData("Base", "Old: %.2f, Adj: %.3f, New: %.2f", basePos, baseAdjustment, newBasePos);
        telemetry.addData("Updown", "Old: %.2f, New: %.2f", updownPos, targetUpdown);
        telemetry.addData("GripperRot", "Old: %.2f, Angle: %.1f, New: %.2f",
                gripperRotPos, targetAngle, gripperAngle);
    }
}
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {

    // Dashboard configurable servo positions
    public static double armPivotPosition = 0.4;
    public static double armRotationPosition = 0.4;
    public static double gripperPosition = 0.5;
    public static double rollPosition = 0.5;

    private Servo armPivot, armRotation, gripper, roll;

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

        // Initialize arm servos
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        armRotation = hardwareMap.get(Servo.class, "armRotation");
        gripper = hardwareMap.get(Servo.class, "gripper");
        roll = hardwareMap.get(Servo.class, "roll");

        // Set initial positions
        armPivot.setPosition(armPivotPosition);
        armRotation.setPosition(armRotationPosition);
        gripper.setPosition(gripperPosition);
        roll.setPosition(rollPosition);

        telemetry.addLine("Arm Test OpMode Ready!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Use FTC Dashboard to adjust servo positions in real-time");
        telemetry.addLine("A button: Apply current dashboard positions");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Run the command scheduler
            CommandScheduler.getInstance().run();

            // Apply dashboard positions to servos instantly
            armPivot.setPosition(armPivotPosition);
            armRotation.setPosition(armRotationPosition);
            gripper.setPosition(gripperPosition);
            roll.setPosition(rollPosition);

            // Display telemetry with all servo positions
            telemetry.addLine("=== SERVO POSITIONS ===");
            telemetry.addData("Arm Pivot", "Target: %.3f | Actual: %.3f", armPivotPosition, armPivot.getPosition());
            telemetry.addData("Arm Rotation", "Target: %.3f | Actual: %.3f", armRotationPosition, armRotation.getPosition());
            telemetry.addData("Gripper", "Target: %.3f | Actual: %.3f", gripperPosition, gripper.getPosition());
            telemetry.addData("Roll", "Target: %.3f | Actual: %.3f", rollPosition, roll.getPosition());
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addData("A Button", gamepad1.a ? "Pressed (Applying Positions)" : "Not Pressed");
            telemetry.addLine();
            telemetry.addLine("Adjust servo positions using FTC Dashboard variables:");
            telemetry.addLine("- armPivotPosition");
            telemetry.addLine("- armRotationPosition");
            telemetry.addLine("- gripperPosition");
            telemetry.addLine("- rollPosition");
            telemetry.update();
        }
    }
}

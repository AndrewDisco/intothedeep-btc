package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
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

        DcMotor[] motors = new DcMotor[4];
        String[] names = {"0", "1", "2", "3"};
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();

        int selected = 0;
        boolean motorOn = false;
        boolean dpadUpPrev = false, dpadDownPrev = false, aPrev = false;

        telemetry.addLine("Use D-pad up/down to select motor. Press A to toggle power.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Run the command scheduler
            CommandScheduler.getInstance().run();

            // Handle D-pad up/down for selection
            if (gamepad1.dpad_up && !dpadUpPrev) {
                selected = (selected + 1) % 4;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                selected = (selected + 3) % 4; // +3 mod 4 is -1 mod 4
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            // Handle A button to toggle motor
            if (gamepad1.a && !aPrev) {
                motorOn = !motorOn;
            }
            aPrev = gamepad1.a;

            // Set motor powers
            for (int i = 0; i < 4; i++) {
                motors[i].setPower((i == selected && motorOn) ? 0.5 : 0.0);
            }

            // Dashboard and telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Selected Motor", names[selected]);
            packet.put("Power", motorOn ? 0.5 : 0.0);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Selected Motor", names[selected]);
            telemetry.addData("Power", motorOn ? 0.5 : 0.0);
            telemetry.update();

            sleep(50);
        }

        // Stop all motors at end
        for (DcMotor motor : motors) {
            motor.setPower(0.0);
        }
    }
}
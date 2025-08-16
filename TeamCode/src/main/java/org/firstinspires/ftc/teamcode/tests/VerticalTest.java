package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

@Config
@TeleOp(name = " glisisere verticale ", group = "Test")
public class VerticalTest extends LinearOpMode {

    public static double TARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        VerticalExtension verticalExtension = new VerticalExtension(hardwareMap);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);

        waitForStart();

        while (opModeIsActive()) {

            verticalExtension.setTargetPosition(TARGET);
            CommandScheduler.getInstance().run();

            telemetry.addData("target", TARGET);
            telemetry.addData("pos", verticalExtension.getCurrentPosition());
            telemetry.update();
        }
    }
}
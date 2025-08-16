package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

@Config
@TeleOp(name = " ridicare  ", group = "Test")
public class RidicationTest extends LinearOpMode {

    public static double TARGET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        HorizontalSlider horizontalSlider = new HorizontalSlider(hardwareMap);
        VerticalExtension verticalExtension = new VerticalExtension(hardwareMap);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);

        horizontalSlider.setTargetPosition(0d);

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
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

@Config
@TeleOp(name = " ridicare  ", group = "Test")
public class RidicationTest extends LinearOpMode {

    public static double TARGET = 0;
    public static double PTO_TARGET = 0;

    public static double rf_POWER=0, rb_POWER=0;

    //0.3 decuplat
    //0 cuplat

    //26000 target glisiere

    Servo pto;

    DcMotorEx rf, rb;

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
            pto = hardwareMap.get(Servo.class, "pto");
            pto.setPosition(PTO_TARGET);

            rf = hardwareMap.get(DcMotorEx.class, "0");
            rb = hardwareMap.get(DcMotorEx.class, "1");

            rf.setPower(rf_POWER);
            rb.setPower(rb_POWER);

            telemetry.addData("target", TARGET);
            telemetry.addData("pos", verticalExtension.getCurrentPosition());
            telemetry.update();
        }
    }
}
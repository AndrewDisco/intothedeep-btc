package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "test encoder glisisere", group = "Test")
public class SliderEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "glisiere");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("pos", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}

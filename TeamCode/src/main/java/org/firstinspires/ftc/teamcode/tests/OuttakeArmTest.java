package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;

@Config
@TeleOp(name = "outtake arm", group = "Test")
public class OuttakeArmTest extends LinearOpMode {

    public static double ARM = 0.3;
    public static double PITCH = 0.3;
    public static double ROLL = 0.5;
    public static double GRIPPER = 0.4;

    // arm 0.9
    // pitch 0


    @Override
    public void runOpMode() throws InterruptedException {
        OuttakeArm outtakeArm = new OuttakeArm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            outtakeArm.setArm(ARM);
            outtakeArm.setPitch(PITCH);
            outtakeArm.setRoll(ROLL);
            outtakeArm.setGripper(GRIPPER);
        }
    }
}

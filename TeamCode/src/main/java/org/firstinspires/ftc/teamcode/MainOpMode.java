package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GripperControlCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.SliderControlCommand;
import org.firstinspires.ftc.teamcode.commands.TelemetryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

import java.util.List;

@TeleOp(name = "Main Drive", group = "Competition")
public class MainOpMode extends LinearOpMode {

    private Drivetrain drivetrain;
    private HorizontalSlider horizontalSlider;
    private Gripper gripper;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // Drive speed coefficient
    private static final double DRIVE_SPEED = 1.0;

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

        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap);
        horizontalSlider = new HorizontalSlider(hardwareMap);
        gripper = new Gripper(hardwareMap);

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Initialize arm servos
        Servo armpivot = hardwareMap.get(Servo.class, "armPivot");
        armpivot.setPosition(0.4);

        Servo armrotation = hardwareMap.get(Servo.class, "armRotation");
        armrotation.setPosition(0.22);

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(gripper);

        // Create and schedule commands
        DriveCommand driveCommand = new DriveCommand(drivetrain, driverGamepad, DRIVE_SPEED);
        SliderControlCommand sliderCommand = new SliderControlCommand(horizontalSlider, gamepad2);
        GripperControlCommand gripperCommand = new GripperControlCommand(gripper, operatorGamepad);
        PickupCommand pickupCommand = new PickupCommand(armpivot, armrotation, gripper);
        TelemetryCommand telemetryCommand = new TelemetryCommand(telemetry, drivetrain, horizontalSlider,
                                                                gripper, driverGamepad, operatorGamepad, gamepad2);

        // Schedule the commands as default commands
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, driveCommand);
        CommandScheduler.getInstance().setDefaultCommand(horizontalSlider, sliderCommand);
        CommandScheduler.getInstance().setDefaultCommand(gripper, gripperCommand);
        CommandScheduler.getInstance().schedule(telemetryCommand);

        // Initial telemetry
        telemetry.addLine("Main Drive OpMode Ready!");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 1 - Drive Controls:");
        telemetry.addLine("Left stick: Move forward/back + strafe");
        telemetry.addLine("Right stick X: Rotate");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 2 - Operator Controls:");
        telemetry.addLine("Right trigger: Extend sliders");
        telemetry.addLine("Left trigger: Retract sliders");
        telemetry.addLine("Y button: Toggle gripper open/close");
        telemetry.addLine("Right stick X: Control gripper roll");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update gamepads
            driverGamepad.readButtons();
            operatorGamepad.readButtons();

            if (operatorGamepad.getButton(GamepadKeys.Button.X)) {
                CommandScheduler.getInstance().schedule(pickupCommand);
            }

            // Run the command scheduler
            CommandScheduler.getInstance().run();
        }
    }
}

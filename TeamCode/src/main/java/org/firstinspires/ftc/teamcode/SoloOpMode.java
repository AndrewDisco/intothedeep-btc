package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.SliderControlCommand;
import org.firstinspires.ftc.teamcode.commands.SoloGripperControlCommand;
import org.firstinspires.ftc.teamcode.commands.SoloTelemetryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

import java.util.List;

@TeleOp(name = "Solo Drive", group = "Competition")
public class SoloOpMode extends LinearOpMode {

    private Drivetrain drivetrain;
    private HorizontalSlider horizontalSlider;
    private Gripper gripper;
    private Arm arm;
    private VerticalExtension verticalExtension;
    private OuttakeArm outtakeArm;
    private GamepadEx soloGamepad;

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
        arm = new Arm(hardwareMap);
        verticalExtension = new VerticalExtension(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);

        // Initialize single gamepad
        soloGamepad = new GamepadEx(gamepad1);

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);
        CommandScheduler.getInstance().registerSubsystem(outtakeArm);


        // Create and schedule commands for solo operation
        DriveCommand driveCommand = new DriveCommand(drivetrain, soloGamepad, DRIVE_SPEED);
        SliderControlCommand sliderCommand = new SliderControlCommand(horizontalSlider, gamepad1, arm, gripper);
        SoloGripperControlCommand gripperCommand = new SoloGripperControlCommand(gripper, soloGamepad);
        SoloTelemetryCommand telemetryCommand = new SoloTelemetryCommand(telemetry, drivetrain, horizontalSlider,
                                                                gripper, soloGamepad, gamepad1, arm);
        OuttakeCommand outtakeCommand = new OuttakeCommand(verticalExtension, outtakeArm);
        IntakeSpecimenCommand intakeSpecimenCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);

        // Schedule the commands as default commands
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, driveCommand);
        CommandScheduler.getInstance().setDefaultCommand(horizontalSlider, sliderCommand);
        CommandScheduler.getInstance().setDefaultCommand(gripper, gripperCommand);
        CommandScheduler.getInstance().schedule(telemetryCommand);

        // Button mappings for single gamepad
        soloGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PickupCommand(gripper, arm));
        soloGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DropCommand(gripper, arm));
        soloGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(outtakeArm::close),
                        new InstantCommand(outtakeArm::open),
                        () -> outtakeArm.isOpen
                )
        );
        soloGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                intakeSpecimenCommand,
                outtakeCommand,
                () -> verticalExtension.getCurrentPosition() >= 5000
        ));

        // Initial telemetry
        telemetry.addLine("Solo Drive OpMode Ready!");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 1 - ALL Controls:");
        telemetry.addLine("Left stick: Move forward/back + strafe");
        telemetry.addLine("Right stick X: Rotate");
        telemetry.addLine("Right trigger: Extend sliders");
        telemetry.addLine("Left trigger: Retract sliders");
        telemetry.addLine("Y button: Toggle gripper open/close");
        telemetry.addLine("X button: Execute pickup sequence");
        telemetry.addLine("B button: Execute drop sequence");
        telemetry.addLine("Dpad Right: Gripper rotate right");
        telemetry.addLine("Dpad Left: Gripper rotate left");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update command scheduler
            CommandScheduler.getInstance().run();

            // Periodic telemetry updates are handled by TelemetryCommand
        }
    }
}

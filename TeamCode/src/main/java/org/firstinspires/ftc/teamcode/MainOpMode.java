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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.GripperControlCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.SliderControlCommand;
import org.firstinspires.ftc.teamcode.commands.TelemetryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

import java.util.List;

@TeleOp(name = "Main Drive", group = "Competition")
public class MainOpMode extends LinearOpMode {

    private Drivetrain drivetrain;
    private HorizontalSlider horizontalSlider;
    private Gripper gripper;
    private Arm arm;
    private VerticalExtension verticalExtension;
    private OuttakeArm outtakeArm;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Servo pto;

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

        // Initialize PTO servo
        pto = hardwareMap.get(Servo.class, "pto");

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(arm);
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);
        CommandScheduler.getInstance().registerSubsystem(outtakeArm);

        // Create and schedule commands
        DriveCommand driveCommand = new DriveCommand(drivetrain, driverGamepad, DRIVE_SPEED);
        SliderControlCommand sliderCommand = new SliderControlCommand(horizontalSlider, gamepad2, arm, gripper);
        GripperControlCommand gripperCommand = new GripperControlCommand(gripper, operatorGamepad);
        TelemetryCommand telemetryCommand = new TelemetryCommand(telemetry, drivetrain, horizontalSlider,
                                                                gripper, driverGamepad, operatorGamepad, gamepad2,
                                                                arm);
        OuttakeCommand outtakeCommand = new OuttakeCommand(verticalExtension, outtakeArm);
        IntakeSpecimenCommand intakeSpecimenCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);

        // Schedule the commands as default commands
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, driveCommand);
        CommandScheduler.getInstance().setDefaultCommand(horizontalSlider, sliderCommand);
        CommandScheduler.getInstance().setDefaultCommand(gripper, gripperCommand);
        CommandScheduler.getInstance().schedule(telemetryCommand);

        // Button mappings for operator gamepad
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PickupCommand(gripper, arm));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DropCommand(gripper, arm));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(outtakeArm::close),
                        new InstantCommand(outtakeArm::open),
                        () -> outtakeArm.isOpen
                )
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                intakeSpecimenCommand,
                outtakeCommand,
                () -> verticalExtension.getCurrentPosition() >= 5000
        ));

        // PTO controls for operator gamepad
        pto.setPosition(0.3);

        // Set initial positions
        outtakeArm.goToInit();
        arm.goToInit();

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> pto.setPosition(0))
        );
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> pto.setPosition(0.3))
        );

        // Button mappings for driver gamepad (same controls except gripper rotation)
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new PickupCommand(gripper, arm));
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new DropCommand(gripper, arm));
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(new InstantCommand(() -> {
                    if (gripper.isOpen) {
                        gripper.close();
                    } else {
                        gripper.open();
                    }
                }));
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ConditionalCommand(
                        new InstantCommand(outtakeArm::close),
                        new InstantCommand(outtakeArm::open),
                        () -> outtakeArm.isOpen
                )
        );
        driverGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new IntakeSpecimenCommand(verticalExtension, outtakeArm),
                new OuttakeCommand(verticalExtension, outtakeArm),
                () -> verticalExtension.getCurrentPosition() >= 5000
        ));

        // Initial telemetry
        telemetry.addLine("Main Drive OpMode Ready!");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 1 - Drive + Button Controls:");
        telemetry.addLine("Left stick: Move forward/back + strafe");
        telemetry.addLine("Right stick X: Rotate");
        telemetry.addLine("Y button: Toggle gripper open/close");
        telemetry.addLine("X button: Execute pickup sequence");
        telemetry.addLine("B button: Execute drop sequence");
        telemetry.addLine("A button: Outtake/Intake specimen (conditional)");
        telemetry.addLine("Dpad Up: Toggle outtake arm open/close");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD 2 - Operator Controls:");
        telemetry.addLine("Right trigger: Extend sliders");
        telemetry.addLine("Left trigger: Retract sliders");
        telemetry.addLine("Y button: Toggle gripper open/close");
        telemetry.addLine("X button: Execute pickup sequence");
        telemetry.addLine("B button: Execute drop sequence");
        telemetry.addLine("A button: Outtake/Intake specimen (conditional)");
        telemetry.addLine("Dpad Up: Toggle outtake arm open/close");
        telemetry.addLine("Right stick X: Control gripper roll");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update gamepads
            driverGamepad.readButtons();
            operatorGamepad.readButtons();

            // Run the command scheduler
            CommandScheduler.getInstance().run();
        }
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class SoloTelemetryCommand extends CommandBase {
    private final Telemetry telemetry;
    private final HorizontalSlider horizontalSlider;
    private final Gripper gripper;
    private final GamepadEx soloGamepad;
    private final Gamepad gamepad1;
    private final Arm arm;

    public SoloTelemetryCommand(Telemetry telemetry, Drivetrain drivetrain,
                           HorizontalSlider horizontalSlider, Gripper gripper,
                           GamepadEx soloGamepad, Gamepad gamepad1, Arm arm) {
        this.telemetry = telemetry;
        this.horizontalSlider = horizontalSlider;
        this.gripper = gripper;
        this.soloGamepad = soloGamepad;
        this.gamepad1 = gamepad1;
        this.arm = arm;
    }

    @Override
    public void execute() {
        // Drive telemetry
        double x = -soloGamepad.getLeftX();
        double y = -soloGamepad.getLeftY();
        double rx = -soloGamepad.getRightX();

        telemetry.addData("Drive Power", "X: %.2f, Y: %.2f, RX: %.2f", x, y, rx);
        telemetry.addLine();

        // Slider telemetry
        telemetry.addData("Slider Target", "%.0f", horizontalSlider.getTargetPosition());
        telemetry.addData("Slider Current", "%.0f", horizontalSlider.getCurrentPosition());
        telemetry.addData("Slider Error", "%.0f", horizontalSlider.getPositionError());
        telemetry.addData("Slider Motor Power", "%.3f", horizontalSlider.getMotorPower());
        telemetry.addData("Slider At Target", horizontalSlider.atTargetPosition());
        telemetry.addLine();

        // Gripper telemetry
        telemetry.addData("Gripper State", gripper.isOpen ? "OPEN" : "CLOSED");
        telemetry.addLine();

        // Control inputs
        telemetry.addData("Right Trigger", "%.3f", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", "%.3f", gamepad1.left_trigger);
        telemetry.addData("Y Button", soloGamepad.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y));
        telemetry.addData("Dpad Left", soloGamepad.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT));
        telemetry.addData("Dpad Right", soloGamepad.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT));

        telemetry.update();
    }
}

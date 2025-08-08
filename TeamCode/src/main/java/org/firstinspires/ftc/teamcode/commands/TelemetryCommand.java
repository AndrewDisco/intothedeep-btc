package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class TelemetryCommand extends CommandBase {
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private final HorizontalSlider horizontalSlider;
    private final Gripper gripper;
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;
    private final Gamepad gamepad2;

    public TelemetryCommand(Telemetry telemetry, Drivetrain drivetrain,
                           HorizontalSlider horizontalSlider, Gripper gripper,
                           GamepadEx driverGamepad, GamepadEx operatorGamepad,
                           Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
        this.horizontalSlider = horizontalSlider;
        this.gripper = gripper;
        this.driverGamepad = driverGamepad;
        this.operatorGamepad = operatorGamepad;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void execute() {
        // Drive telemetry
        double x = -driverGamepad.getLeftX();
        double y = -driverGamepad.getLeftY();
        double rx = -driverGamepad.getRightX();

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
        telemetry.addData("Gripper Roll Input", "%.2f", operatorGamepad.getRightX());
        telemetry.addLine();

        // Control inputs
        telemetry.addData("Gamepad 2 - Right Trigger", "%.2f", gamepad2.right_trigger);
        telemetry.addData("Gamepad 2 - Left Trigger", "%.2f", gamepad2.left_trigger);
        telemetry.addData("Gamepad 2 - Y Button", operatorGamepad.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y) ? "Pressed" : "Not Pressed");
        telemetry.addLine();

        // Quick reference
        telemetry.addLine("Controls:");
        telemetry.addLine("GP1: Left stick: Move/Strafe | Right stick X: Rotate");
        telemetry.addLine("GP2: Triggers: Sliders | Y: Gripper | Right stick X: Roll");

        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}

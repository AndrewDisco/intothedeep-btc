package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;

public class SliderControlCommand extends CommandBase {
    private final HorizontalSlider horizontalSlider;
    private final Gamepad gamepad;

    private static final double TRIGGER_DEADZONE = 0.1;
    private boolean prevRightTriggerPressed = false;
    private boolean prevLeftTriggerPressed = false;

    public SliderControlCommand(HorizontalSlider horizontalSlider, Gamepad gamepad) {
        this.horizontalSlider = horizontalSlider;
        this.gamepad = gamepad;

        addRequirements(horizontalSlider);
    }

    @Override
    public void execute() {
        double rightTrigger = gamepad.right_trigger;
        double leftTrigger = gamepad.left_trigger;

        // Current pressed states
        boolean rightPressed = rightTrigger > TRIGGER_DEADZONE;
        boolean leftPressed = leftTrigger > TRIGGER_DEADZONE;

        // Only fire on the rising edge of right trigger
        if (rightPressed && !prevRightTriggerPressed) {
            if (horizontalSlider.getTargetPosition() == 0) {
                horizontalSlider.setTargetPosition(17500);
            } else {
                horizontalSlider.setTargetPosition(0);
            }
        }
        // Only fire on the rising edge of left trigger
        else if (leftPressed && !prevLeftTriggerPressed) {
            if (horizontalSlider.getTargetPosition() == 0) {
                horizontalSlider.setTargetPosition(35000);
            } else {
                horizontalSlider.setTargetPosition(0);
            }
        }

        // Update previous states for next loop
        prevRightTriggerPressed = rightPressed;
        prevLeftTriggerPressed = leftPressed;
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}

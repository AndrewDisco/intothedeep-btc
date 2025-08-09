package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import com.qualcomm.robotcore.hardware.Servo;

public class SliderControlCommand extends CommandBase {
    private final HorizontalSlider horizontalSlider;
    private final Gamepad gamepad;
    private final Servo armPivot;

    private static final double TRIGGER_DEADZONE = 0.1;
    private static final double ARM_PIVOT_INITIAL = 0.7;
    private static final double ARM_PIVOT_RIGHT_TRIGGER = 0.36;
    private static final double ARM_PIVOT_LEFT_TRIGGER = 0.38;

    private boolean prevRightTriggerPressed = false;
    private boolean prevLeftTriggerPressed = false;
    private double currentArmPivotPosition = ARM_PIVOT_INITIAL;

    public SliderControlCommand(HorizontalSlider horizontalSlider, Gamepad gamepad, Servo armPivot) {
        this.horizontalSlider = horizontalSlider;
        this.gamepad = gamepad;
        this.armPivot = armPivot;

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
                currentArmPivotPosition = ARM_PIVOT_RIGHT_TRIGGER;
            } else {
                horizontalSlider.setTargetPosition(0);
                currentArmPivotPosition = ARM_PIVOT_INITIAL;
            }
        }
        // Only fire on the rising edge of left trigger
        else if (leftPressed && !prevLeftTriggerPressed) {
            if (horizontalSlider.getTargetPosition() == 0) {
                horizontalSlider.setTargetPosition(35000);
                currentArmPivotPosition = ARM_PIVOT_LEFT_TRIGGER;
            } else {
                horizontalSlider.setTargetPosition(0);
                currentArmPivotPosition = ARM_PIVOT_INITIAL;
            }
        }

        // Update the arm pivot servo position
        armPivot.setPosition(currentArmPivotPosition);

        // Update previous states for next loop
        prevRightTriggerPressed = rightPressed;
        prevLeftTriggerPressed = leftPressed;
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}

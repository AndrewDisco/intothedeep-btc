package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class SliderControlCommand extends CommandBase {
    private final HorizontalSlider horizontalSlider;
    private final Gamepad gamepad;
    private final Arm arm;
    private final Gripper gripper;

    private static final double TRIGGER_DEADZONE = 0.1;

    private boolean prevRightTriggerPressed = false;
    private boolean prevLeftTriggerPressed = false;
    private MoveSampleInGripperCommand moveSampleInGripperCommand;
    private IntakeArmCommand intakeArmCommand;

    public SliderControlCommand(HorizontalSlider horizontalSlider, Gamepad gamepad, Arm arm, Gripper gripper) {
        this.horizontalSlider = horizontalSlider;
        this.gamepad = gamepad;
        this.arm = arm;
        this.gripper = gripper;

        moveSampleInGripperCommand = new MoveSampleInGripperCommand(gripper, arm);
        intakeArmCommand = new IntakeArmCommand(arm, gripper);

        addRequirements(horizontalSlider);
        // Note: NOT requiring arm or gripper subsystems so other commands can take control
    }

    @Override
    public void execute() {
        double rightTrigger = gamepad.right_trigger;
        double leftTrigger = gamepad.left_trigger;

        // Current pressed states
        boolean rightPressed = rightTrigger > TRIGGER_DEADZONE;
        boolean leftPressed = leftTrigger > TRIGGER_DEADZONE;

        Trigger extended = new Trigger(() -> horizontalSlider.getCurrentPosition() >= 15000);
        Trigger retracted = new Trigger(() -> horizontalSlider.getCurrentPosition() < 15000);


        // Only fire on the rising edge of right trigger
        if (rightPressed && !prevRightTriggerPressed) {
            if (horizontalSlider.getTargetPosition() == 0) {
                // Extending sliders
                horizontalSlider.setTargetPosition(17500);
                intakeArmCommand.schedule();

            } else {
                // Retracting sliders - close gripper and move arm back
                horizontalSlider.setTargetPosition(0);
                moveSampleInGripperCommand.schedule();
            }
        }
        // Only fire on the rising edge of left trigger
        else if (leftPressed && !prevLeftTriggerPressed) {
            if (horizontalSlider.getTargetPosition() == 0) {
                // Extending sliders from 0 to 35000 - open gripper and move arm
                horizontalSlider.setTargetPosition(35000);
                intakeArmCommand.schedule();


            } else if (horizontalSlider.getTargetPosition() == 17500) {
                // Extending sliders from 17500 to 35000 - move arm to left trigger position
                horizontalSlider.setTargetPosition(35000);
                // Keep gripper open (already open from previous position)
            } else {
                // Retracting sliders from any other position - close gripper and move arm back
                horizontalSlider.setTargetPosition(0);
                moveSampleInGripperCommand.schedule();
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

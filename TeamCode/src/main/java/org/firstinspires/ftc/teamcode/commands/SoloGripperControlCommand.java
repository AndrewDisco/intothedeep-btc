package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class SoloGripperControlCommand extends CommandBase {
    private final Gripper gripper;
    private final GamepadEx gamepad;

    private static final double LEFT_POSITION = 0.4;   // Left rotation position
    private static final double RIGHT_POSITION = 1.0;  // Right rotation position
    private static final double MIDDLE_POSITION = 0.7; // Default middle position

    private boolean prevYButtonPressed = false;
    private boolean prevDpadLeftPressed = false;
    private boolean prevDpadRightPressed = false;

    public SoloGripperControlCommand(Gripper gripper, GamepadEx gamepad) {
        this.gripper = gripper;
        this.gamepad = gamepad;

        addRequirements(gripper);
    }

    @Override
    public void execute() {
        // Y button to toggle gripper open/close
        boolean yButtonPressed = gamepad.getButton(GamepadKeys.Button.Y);

        if (yButtonPressed && !prevYButtonPressed) {
            if (gripper.isOpen) {
                gripper.close();
            } else {
                gripper.open();
            }
        }

        prevYButtonPressed = yButtonPressed;

        // Dpad controls for gripper rotation
        boolean dpadLeftPressed = gamepad.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRightPressed = gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT);

        // Set gripper rotation based on dpad input
        if (dpadLeftPressed && !prevDpadLeftPressed) {
            gripper.setRollPosition(LEFT_POSITION);
        } else if (dpadRightPressed && !prevDpadRightPressed) {
            gripper.setRollPosition(RIGHT_POSITION);
        } else if (!dpadLeftPressed && !dpadRightPressed) {
            // Return to middle position when no dpad is pressed
            gripper.setRollPosition(MIDDLE_POSITION);
        }

        prevDpadLeftPressed = dpadLeftPressed;
        prevDpadRightPressed = dpadRightPressed;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

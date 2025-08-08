package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class GripperControlCommand extends CommandBase {
    private final Gripper gripper;
    private final GamepadEx gamepad;

    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double MIDDLE_ROLL_POSITION = 0.7;
    private boolean prevYButtonPressed = false;

    public GripperControlCommand(Gripper gripper, GamepadEx gamepad) {
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

        // Right joystick X for gripper roll control
        double rightStickX = gamepad.getRightX();

        if (Math.abs(rightStickX) > JOYSTICK_DEADZONE) {
            // Convert joystick input (-1 to 1) to angle (-90 to 90 degrees)
            double angle = rightStickX * 90;
            gripper.turn(angle);
        } else {
            // Return to middle position when joystick is not in use
            gripper.setRollPosition(MIDDLE_ROLL_POSITION);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}

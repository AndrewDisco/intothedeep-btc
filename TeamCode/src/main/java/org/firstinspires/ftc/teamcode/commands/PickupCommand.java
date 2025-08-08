package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class PickupCommand extends CommandBase {
    private final Servo armPivot;
    private final Servo armRotation;
    private final Gripper gripper;

    // Original positions to return to
    private final double originalPivotPosition;
    private final double originalRotationPosition;

    // Pickup positions
    private static final double PICKUP_PIVOT_POSITION = 0.17;
    private static final double PICKUP_ROTATION_POSITION = 0.22;

    // State machine for command sequence
    private enum PickupState {
        MOVING_TO_PICKUP,
        GRIPPING,
        RETURNING_TO_ORIGINAL,
        FINISHED
    }

    private PickupState currentState;
    private ElapsedTime timer;
    private static final double MOVEMENT_DELAY = 0.5; // 500ms delay for arm movement
    private static final double GRIP_DELAY = 0.3; // 300ms delay before gripping

    public PickupCommand(Servo armPivot, Servo armRotation, Gripper gripper) {
        this.armPivot = armPivot;
        this.armRotation = armRotation;
        this.gripper = gripper;

        // Store original positions when command is created
        this.originalPivotPosition = armPivot.getPosition();
        this.originalRotationPosition = armRotation.getPosition();

        this.timer = new ElapsedTime();

        addRequirements(gripper);
    }

    @Override
    public void initialize() {
        currentState = PickupState.MOVING_TO_PICKUP;
        timer.reset();

        // Move arm to pickup position
        armPivot.setPosition(PICKUP_PIVOT_POSITION);
        armRotation.setPosition(PICKUP_ROTATION_POSITION);
    }

    @Override
    public void execute() {
        switch (currentState) {
            case MOVING_TO_PICKUP:
                if (timer.seconds() >= MOVEMENT_DELAY) {
                    // Arm should be in position, now close gripper
                    gripper.close();
                    currentState = PickupState.GRIPPING;
                    timer.reset();
                }
                break;

            case GRIPPING:
                if (timer.seconds() >= GRIP_DELAY) {
                    // Gripper is closed, now return to original position
                    armPivot.setPosition(originalPivotPosition);
                    armRotation.setPosition(originalRotationPosition);
                    currentState = PickupState.RETURNING_TO_ORIGINAL;
                    timer.reset();
                }
                break;

            case RETURNING_TO_ORIGINAL:
                if (timer.seconds() >= MOVEMENT_DELAY) {
                    // Movement complete
                    currentState = PickupState.FINISHED;
                }
                break;

            case FINISHED:
                // Command is complete
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == PickupState.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // If interrupted, return to original position
            armPivot.setPosition(originalPivotPosition);
            armRotation.setPosition(originalRotationPosition);
        }
    }
}

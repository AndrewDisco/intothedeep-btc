package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {

    private final Servo armPivot;
    private final Servo armRotation;

    public static final double PIVOT_INITIAL = 0.7;
    public static final double PIVOT_PICKUP = 0.28;
    public static final double PIVOT_RIGHT_TRIGGER = 0.36;
    public static final double PIVOT_LEFT_TRIGGER = 0.38;

    public static final double ROTATION_INITIAL = 0.7;

    public Arm(HardwareMap hardwareMap) {
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        armRotation = hardwareMap.get(Servo.class, "armRotation");

        // Set initial positions
        armPivot.setPosition(PIVOT_INITIAL);
        armRotation.setPosition(ROTATION_INITIAL);
    }

    public void setPivotPosition(double position) {
        armPivot.setPosition(position);
    }

    public void setRotationPosition(double position) {
        armRotation.setPosition(position);
    }

    public void setToInitialPosition() {
        armPivot.setPosition(PIVOT_INITIAL);
        armRotation.setPosition(ROTATION_INITIAL);
    }

    public void setToPickupPosition() {
        armPivot.setPosition(PIVOT_PICKUP);
    }

    public void setToRightTriggerPosition() {
        armPivot.setPosition(PIVOT_RIGHT_TRIGGER);
    }

    public void setToLeftTriggerPosition() {
        armPivot.setPosition(PIVOT_LEFT_TRIGGER);
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {

    private final Servo armPivot;
    private final Servo armRotation;

    public static final double PIVOT_INITIAL = 0.7;
    public static final double PIVOT_INTAKE = 0.4;
    public static final double PIVOT_PICKUP = 0.33;
    public static final double PIVOT_DROPOFF = 0.7;

    public static final double ROTATION_INITIAL = 0.665;
    public static final double ROTATION_DROPOFF = 0.05;

    public Arm(HardwareMap hardwareMap) {
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        armRotation = hardwareMap.get(Servo.class, "armRotation");

        // Set initial positions
        armPivot.setPosition(PIVOT_INITIAL);
        armRotation.setPosition(ROTATION_INITIAL);
    }

    public void set(double pivot, double rotation) {
        armPivot.setPosition(pivot);
        armRotation.setPosition(rotation);
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
        armRotation.setPosition(ROTATION_INITIAL);
    }

}

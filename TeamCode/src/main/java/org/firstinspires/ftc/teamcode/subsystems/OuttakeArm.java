package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeArm extends SubsystemBase {

    static public double ARM_INTAKE = 0.06;
    static public double ARM_OUTTAKE = 0.85;
    static public double PITCH_INTAKE = 0.23;
    static public double PITCH_OUTTAKE = 0.1;
    static public double ROLL_INTAKE = 0.4;
    static public double ROLL_OUTTAKE = 0.45;
    static public double OPEN = 0.45;
    static public double CLOSED = 0.67;

    Servo s1, s2, pitch, gripper, roll;

    public OuttakeArm(HardwareMap hardwareMap) {
        s1 = hardwareMap.get(Servo.class, "o1");
        s2 = hardwareMap.get(Servo.class, "o2");
        pitch = hardwareMap.get(Servo.class, "pitch");
        roll = hardwareMap.get(Servo.class, "oroll");
        gripper = hardwareMap.get(Servo.class, "og");

        goToIntake();
        close();
    }

    public void setArm(double pos) {
        s1.setPosition(pos);
        s2.setPosition(1.0 - pos);
    }

    public void setPitch(double pos) {
        pitch.setPosition(pos);
    }

    public void setRoll(double pos) {
        roll.setPosition(pos);
    }

    public void setGripper(double pos) {
        gripper.setPosition(pos);
    }


    public boolean isOpen = false;
    public void close() {
        gripper.setPosition(CLOSED);
        isOpen = false;
    }
    public void open() {
        gripper.setPosition(OPEN);
        isOpen = true;
    }

    public void goToIntake() {
        setArm(ARM_INTAKE);
        setRoll(ROLL_INTAKE);
        setPitch(PITCH_INTAKE);
    }
    public void goToOuttake() {
        setArm(ARM_OUTTAKE);
        setRoll(ROLL_OUTTAKE);
        setPitch(PITCH_OUTTAKE);
    }


}

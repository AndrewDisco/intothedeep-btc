package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtendoModule {
    HardwareMap hardwareMap;
    public ExtendoModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    static double kp = 0.001, ki = 0.00, kd = 0.000015;
    DcMotorEx motor;
    PIDController controller = new PIDController(kp, ki, kd);

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "glisiere");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller.reset();

        controller.setSetPoint(-2);
    }

    public void init_teleOP() {
        motor = hardwareMap.get(DcMotorEx.class, "glisiere");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller.setSetPoint(-2);
    }

    public void update() {
        controller.setPID(kp, ki, kd);
        if (!controller.atSetPoint() || controller.getSetPoint() != motor.getCurrentPosition()) {
            double output = controller.calculate(
                    motor.getCurrentPosition()
            );
            motor.setPower(output);

        }
    }

    public void acasa() {
        controller.setSetPoint(0);
    }

    public double getControllerPosition() {
        return controller.getSetPoint();
    }
    public int getEncoderPosition() {return motor.getCurrentPosition();}

}
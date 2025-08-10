package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.Servo;

public class MoveArmPivotCommand extends InstantCommand {

    public MoveArmPivotCommand(Servo armPivot, double position) {
        super(() -> armPivot.setPosition(position));
    }
}

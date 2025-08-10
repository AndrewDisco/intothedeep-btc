package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class MoveArmCommand extends InstantCommand {

    public MoveArmCommand(Arm arm, double pivotPosition) {
        super(() -> arm.setPivotPosition(pivotPosition), arm);
    }
}

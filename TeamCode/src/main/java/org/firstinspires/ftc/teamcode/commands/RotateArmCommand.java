package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class RotateArmCommand extends InstantCommand {

    public RotateArmCommand(Arm arm, double rotationPosition) {
        super(() -> arm.setRotationPosition(rotationPosition), arm);
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class OpenGripperCommand extends InstantCommand {

    public OpenGripperCommand(Gripper gripper) {
        super(() -> gripper.open(), gripper);
    }
}

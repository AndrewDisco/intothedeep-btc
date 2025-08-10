package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class CloseGripperCommand extends InstantCommand {

    public CloseGripperCommand(Gripper gripper) {
        super(() -> gripper.close(), gripper);
    }
}

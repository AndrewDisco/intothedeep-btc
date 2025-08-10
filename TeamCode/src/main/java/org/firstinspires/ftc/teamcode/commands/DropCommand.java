package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class DropCommand extends SequentialCommandGroup {

    public DropCommand(Gripper gripper, Arm arm) {
        addCommands(
            new MoveArmCommand(arm, 0.75),

            new RotateArmCommand(arm, 0),

            new WaitCommand(300),

            new OpenGripperCommand(gripper),

            new WaitCommand(300),

            new RotateArmCommand(arm, Arm.ROTATION_INITIAL),

            new WaitCommand(300),

            new MoveArmCommand(arm, Arm.PIVOT_INITIAL)
        );
    }
}

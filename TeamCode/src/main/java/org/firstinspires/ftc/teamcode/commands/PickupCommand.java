package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class PickupCommand extends SequentialCommandGroup {

    public PickupCommand(Gripper gripper, Arm arm) {
        addCommands(
            new OpenGripperCommand(gripper),

            new MoveArmCommand(arm, Arm.PIVOT_PICKUP),

            new WaitCommand(200),

            new CloseGripperCommand(gripper),

            new WaitCommand(300),

            new MoveArmCommand(arm, 0.38)
        );
    }
}

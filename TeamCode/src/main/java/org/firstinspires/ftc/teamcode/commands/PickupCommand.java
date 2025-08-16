package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class PickupCommand extends SequentialCommandGroup {

    public PickupCommand(Gripper gripper, Arm arm) {
        addCommands(
            new InstantCommand(gripper::open),

            new MoveArmCommand(arm, Arm.PIVOT_PICKUP),

            new WaitCommand(200),

            new InstantCommand(gripper::close),

            new WaitCommand(200),

            new MoveArmCommand(arm, Arm.PIVOT_INTAKE)
        );
    }
}

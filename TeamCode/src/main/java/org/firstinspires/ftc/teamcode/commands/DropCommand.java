package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class DropCommand extends SequentialCommandGroup {

    public DropCommand(Gripper gripper, Arm arm) {
        addCommands(
            new MoveArmCommand(arm, Arm.PIVOT_DROPOFF),

            new RotateArmCommand(arm, Arm.ROTATION_DROPOFF),
            new InstantCommand(() -> gripper.turn(90)),

            new WaitCommand(200),

            new InstantCommand(gripper::open),

            new WaitCommand(300),
            new InstantCommand(() -> gripper.turn(0)),

            new RotateArmCommand(arm, Arm.ROTATION_INITIAL),

            new WaitCommand(300),

            new MoveArmCommand(arm, Arm.PIVOT_INITIAL)
        );
    }
}

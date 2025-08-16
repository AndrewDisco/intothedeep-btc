package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class MoveSampleInGripperCommand extends SequentialCommandGroup {

    public MoveSampleInGripperCommand(Gripper gripper, Arm arm) {
        addCommands(
                new MoveArmCommand(arm, Arm.PIVOT_INITIAL),
                new InstantCommand(() -> gripper.turn(180)),
                new WaitCommand(100),
                new InstantCommand(gripper::semiclose),
                new WaitCommand(300),
                new InstantCommand(gripper::close),
                new WaitCommand(200),
                new InstantCommand(() -> gripper.turn(0)),
                new WaitCommand(200),
                new RotateArmCommand(arm, 0.25)
            );
    }
}

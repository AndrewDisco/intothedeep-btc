package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class IntakeArmCommand extends SequentialCommandGroup {

    public IntakeArmCommand(Arm arm, Gripper gripper) {
        addCommands(
                new RotateArmCommand(arm, Arm.ROTATION_INITIAL),
                new WaitCommand(200),
                new MoveArmCommand(arm, Arm.PIVOT_INTAKE),
                new InstantCommand(gripper::open),
                new InstantCommand(() -> gripper.turn(0))
        );
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

public class OuttakeCommand extends SequentialCommandGroup {

    public OuttakeCommand(VerticalExtension verticalExtension, OuttakeArm outtakeArm) {
        addCommands(
                new InstantCommand(outtakeArm::close),
                new WaitCommand(100),
                new InstantCommand(() -> verticalExtension.setTargetPosition(VerticalExtension.OUTTAKE_POSITION)),
                new InstantCommand(outtakeArm::goToOuttake)
        );
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

public class IntakeSpecimenCommand extends SequentialCommandGroup {

    public IntakeSpecimenCommand(VerticalExtension verticalExtension, OuttakeArm outtakeArm) {
        addCommands(
                new InstantCommand(outtakeArm::open),
                new WaitCommand(100),
                new InstantCommand(outtakeArm::goToIntake),
                new WaitCommand(200),
                new InstantCommand(() -> verticalExtension.setTargetPosition(VerticalExtension.INTAKE_POSITION))
        );
    }
}

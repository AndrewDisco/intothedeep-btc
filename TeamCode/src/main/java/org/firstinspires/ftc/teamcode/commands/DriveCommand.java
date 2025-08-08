package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final GamepadEx gamepad;
    private final double speedCoefficient;

    public DriveCommand(Drivetrain drivetrain, GamepadEx gamepad, double speedCoefficient) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
        this.speedCoefficient = speedCoefficient;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double x = -gamepad.getLeftX();      // Strafe left/right
        double y = -gamepad.getLeftY();     // Forward/backward (inverted)
        double rx = -gamepad.getRightX();    // Rotation

        drivetrain.driveRobotCentric(x, y, rx, speedCoefficient);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}

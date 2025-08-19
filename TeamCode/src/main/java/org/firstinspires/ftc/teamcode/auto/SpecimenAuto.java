package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.PickupCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;

/**
 * Autonomous mode for specimen scoring using Pedro Pathing
 *
 * Sequence:
 * 1. Move to scoring position 1 and score first specimen (already in outtake arm)
 * 2. Move to pickup position 2, extend horizontal sliders, pickup second specimen
 * 3. Retract sliders and outtake the picked up specimen
 * 4. Move to refinement position 3, extend sliders again for final scoring
 * 5. Retract sliders and final outtake
 */
@Autonomous(name = "Specimen Auto", group = "Main")
public class SpecimenAuto extends OpMode {

    // Pedro Pathing components
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Individual path segments from GeneratedPath
    private PathChain pathToScore1, pathToPickup2, pathToRefine3, pathToEnd;

    // Subsystems
    private HorizontalSlider horizontalSlider;
    private VerticalExtension verticalExtension;
    private OuttakeArm outtakeArm;
    private Gripper gripper;
    private Arm arm;

    // State management
    private int pathState;
    private boolean commandExecuted;
    private SequentialCommandGroup currentCommand;

    // Constants
    private static final double HORIZONTAL_EXTEND_POSITION = 35000;
    private static final double ACTION_TIMEOUT = 3.0; // seconds

    // Auto states
    private enum AutoState {
        MOVE_TO_SCORE_1(0),
        SCORE_FIRST_SPECIMEN(1),
        RETRACT_VERTICAL_AFTER_SCORE(2),
        MOVE_TO_PICKUP_2(3),
        EXTEND_SLIDERS_FOR_PICKUP(4),
        PICKUP_SECOND_SPECIMEN(5),
        RETRACT_SLIDERS_AFTER_PICKUP(6),
        OUTTAKE_SECOND_SPECIMEN(7),
        MOVE_TO_REFINE_3(8),
        EXTEND_SLIDERS_FOR_FINAL(9),
        RETRACT_SLIDERS_FINAL(10),
        FINAL_OUTTAKE(11),
        // Room for additional states
        MOVE_TO_POSITION_4(12),
        ACTION_AT_POSITION_4(13),
        MOVE_TO_POSITION_5(14),
        ACTION_AT_POSITION_5(15),
        PARK(16),
        COMPLETE(-1);

        private final int value;
        AutoState(int value) { this.value = value; }
        public int getValue() { return value; }
    }

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // Initialize subsystems
        initializeSubsystems();

        // Build individual path segments
        buildPathSegments();

        // Set starting pose based on the generated path
        Pose startPose = new Pose(9.000, 63.000, Math.toRadians(0));
        follower.setStartingPose(startPose);

        // Initialize state
        pathState = AutoState.MOVE_TO_SCORE_1.getValue();
        commandExecuted = false;

        // Reset timers
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    /**
     * Build individual path segments from the generated path points
     */
    private void buildPathSegments() {
        // Define poses based on GeneratedPath points
        Pose startPose = new Pose(9.000, 63.000, Math.toRadians(0));
        Pose scorePose1 = new Pose(38.700, 63.000, Math.toRadians(0)); // End of first line
        Pose pickupPose2 = new Pose(16, 17.000, Math.toRadians(0)); // End of curve to scoring
        Pose refinePose3 = new Pose(16, 13.000, Math.toRadians(0)); // Fine positioning
        Pose endPose = new Pose(38.700, 63.000, Math.toRadians(0)); // Return position

        // Build individual paths
        pathToScore1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(startPose),
                        new com.pedropathing.pathgen.Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToPickup2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose1),
                        new com.pedropathing.pathgen.Point(13, 52, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(pickupPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToRefine3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(pickupPose2),
                        new com.pedropathing.pathgen.Point(refinePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToEnd = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(refinePose3),
                        new com.pedropathing.pathgen.Point(21.000, 41.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(endPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private void initializeSubsystems() {
        horizontalSlider = new HorizontalSlider(hardwareMap);
        verticalExtension = new VerticalExtension(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        arm = new Arm(hardwareMap);

        // Register subsystems with CommandScheduler
        CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);
        CommandScheduler.getInstance().registerSubsystem(outtakeArm);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(arm);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(AutoState.MOVE_TO_SCORE_1.getValue());
    }

    @Override
    public void loop() {
        // Update Pedro Pathing
        follower.update();

        // Update command scheduler
        CommandScheduler.getInstance().run();

        // Run autonomous state machine
        autonomousPathUpdate();

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Main autonomous state machine
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // MOVE_TO_SCORE_1
                follower.followPath(pathToScore1, true);
                setPathState(AutoState.SCORE_FIRST_SPECIMEN.getValue());
                break;

            case 1: // SCORE_FIRST_SPECIMEN
                if (!follower.isBusy()) {
                    if (!commandExecuted) {
                        // Create modified IntakeSpecimenCommand without opening outtake arm first
                        currentCommand = createScoreFirstSpecimenCommand();
                        CommandScheduler.getInstance().schedule(currentCommand);
                        actionTimer.resetTimer();
                        commandExecuted = true;
                    }

                    if (currentCommand.isFinished() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        setPathState(AutoState.RETRACT_VERTICAL_AFTER_SCORE.getValue());
                    }
                }
                break;

            case 2: // RETRACT_VERTICAL_AFTER_SCORE
                if (!commandExecuted) {
                    currentCommand = new SequentialCommandGroup(
                        new InstantCommand(() -> verticalExtension.retract()),
                        new InstantCommand(() -> outtakeArm.goToIntake())
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (verticalExtension.atTargetPosition() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.MOVE_TO_PICKUP_2.getValue());
                }
                break;

            case 3: // MOVE_TO_PICKUP_2
                follower.followPath(pathToPickup2, true);
                setPathState(AutoState.EXTEND_SLIDERS_FOR_PICKUP.getValue());
                break;

            case 4: // EXTEND_SLIDERS_FOR_PICKUP
                if (!follower.isBusy()) {
                    if (!commandExecuted) {
                        currentCommand = new SequentialCommandGroup(
                            new InstantCommand(() -> horizontalSlider.setTargetPosition(HORIZONTAL_EXTEND_POSITION))
                        );
                        CommandScheduler.getInstance().schedule(currentCommand);
                        actionTimer.resetTimer();
                        commandExecuted = true;
                    }

                    if (horizontalSlider.atTargetPosition() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        setPathState(AutoState.PICKUP_SECOND_SPECIMEN.getValue());
                    }
                }
                break;

            case 5: // PICKUP_SECOND_SPECIMEN
                if (!commandExecuted) {
                    currentCommand = new PickupCommand(gripper, arm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (currentCommand.isFinished() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.RETRACT_SLIDERS_AFTER_PICKUP.getValue());
                }
                break;

            case 6: // RETRACT_SLIDERS_AFTER_PICKUP
                if (!commandExecuted) {
                    currentCommand = new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlider.retract())
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (horizontalSlider.atTargetPosition() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.OUTTAKE_SECOND_SPECIMEN.getValue());
                }
                break;

            case 7: // OUTTAKE_SECOND_SPECIMEN
                if (!commandExecuted) {
                    currentCommand = new DropCommand(gripper, arm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (currentCommand.isFinished() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.MOVE_TO_REFINE_3.getValue());
                }
                break;

            case 8: // MOVE_TO_REFINE_3
                follower.followPath(pathToRefine3, true);
                setPathState(AutoState.EXTEND_SLIDERS_FOR_FINAL.getValue());
                break;

            case 9: // EXTEND_SLIDERS_FOR_FINAL
                if (!follower.isBusy()) {
                    if (!commandExecuted) {
                        currentCommand = new SequentialCommandGroup(
                            new InstantCommand(() -> horizontalSlider.setTargetPosition(HORIZONTAL_EXTEND_POSITION))
                        );
                        CommandScheduler.getInstance().schedule(currentCommand);
                        actionTimer.resetTimer();
                        commandExecuted = true;
                    }

                    if (horizontalSlider.atTargetPosition() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        setPathState(AutoState.RETRACT_SLIDERS_FINAL.getValue());
                    }
                }
                break;

            case 10: // RETRACT_SLIDERS_FINAL
                if (!commandExecuted) {
                    currentCommand = new SequentialCommandGroup(
                        new InstantCommand(() -> horizontalSlider.retract())
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (horizontalSlider.atTargetPosition() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.FINAL_OUTTAKE.getValue());
                }
                break;

            case 11: // FINAL_OUTTAKE
                if (!commandExecuted) {
                    currentCommand = new DropCommand(gripper, arm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    actionTimer.resetTimer();
                    commandExecuted = true;
                }

                if (currentCommand.isFinished() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                    setPathState(AutoState.COMPLETE.getValue());
                }
                break;

            // Room for additional states
            case 12: // MOVE_TO_POSITION_4
                // Template for additional movement
                // follower.followPath(additionalPath, true);
                setPathState(AutoState.ACTION_AT_POSITION_4.getValue());
                break;

            case 13: // ACTION_AT_POSITION_4
                // Template for additional action
                setPathState(AutoState.MOVE_TO_POSITION_5.getValue());
                break;

            case 14: // MOVE_TO_POSITION_5
                // Template for additional movement
                setPathState(AutoState.ACTION_AT_POSITION_5.getValue());
                break;

            case 15: // ACTION_AT_POSITION_5
                // Template for additional action
                setPathState(AutoState.PARK.getValue());
                break;

            case 16: // PARK
                // Park at final position
                setPathState(AutoState.COMPLETE.getValue());
                break;

            case -1: // COMPLETE
                // Autonomous complete - do nothing
                break;

            default:
                // Unknown state - stop
                setPathState(AutoState.COMPLETE.getValue());
                break;
        }
    }

    /**
     * Creates a modified scoring command that doesn't open the outtake arm first
     * since the specimen is already in the outtake arm
     */
    private SequentialCommandGroup createScoreFirstSpecimenCommand() {
        return new SequentialCommandGroup(
            // Skip the outtake arm opening since specimen is already there
            new InstantCommand(outtakeArm::close),
            new WaitCommand(100),
            new InstantCommand(() -> verticalExtension.setTargetPosition(VerticalExtension.OUTTAKE_POSITION)),
            new InstantCommand(outtakeArm::goToOuttake)
        );
    }

    /**
     * Set the path state and reset timers
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        commandExecuted = false;

        // Cancel any running command when changing states
        if (currentCommand != null && !currentCommand.isFinished()) {
            currentCommand.cancel();
        }
    }

    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("Command Executed", commandExecuted);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Runtime", opmodeTimer.getElapsedTimeSeconds());

        // Subsystem status
        telemetry.addData("Horizontal Position", horizontalSlider.getCurrentPosition());
        telemetry.addData("Horizontal Target", horizontalSlider.getTargetPosition());
        telemetry.addData("Horizontal At Target", horizontalSlider.atTargetPosition());

        // Robot pose
        Pose currentPose = follower.getPose();
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all subsystems
        CommandScheduler.getInstance().reset();
        horizontalSlider.stop();

        // Stop follower
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}

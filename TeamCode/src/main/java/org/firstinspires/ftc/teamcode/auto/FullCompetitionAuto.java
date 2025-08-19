package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DropCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand;
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
 * Comprehensive 13-line autonomous mode with samples and specimens
 *
 * Sequence (30 second total):
 * 1. Score preloaded specimen
 * 2-4. Pickup and score 3 samples
 * 5-12. Pickup and score 4 specimens
 * 13. Park
 */
@Autonomous(name = "Full Competition Auto", group = "Main")
public class FullCompetitionAuto extends OpMode {

    // Pedro Pathing components
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Individual path segments (like SpecimenAuto approach)
    private PathChain pathToScore1, pathToSample1, pathToSample2, pathToSample3;
    private PathChain pathToSpecimen1, pathToScore2, pathToSpecimen2, pathToScore3;
    private PathChain pathToSpecimen3, pathToScore4, pathToSpecimen4, pathToScore5, pathToPark;

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
    private OuttakeCommand outtakeCommand; // Track the outtake command

    // Constants
    private static final double HORIZONTAL_EXTEND_POSITION = 35000;
    private static final double ACTION_TIMEOUT = 2.5; // Reduced for speed
    private static final int TOTAL_SEGMENTS = 13;

    // Auto states - one per path segment
    private enum AutoState {
        // Preloaded specimen scoring
        MOVE_TO_SCORE_1(0),
        SCORE_PRELOAD(1),
        RETRACT_AFTER_PRELOAD(2),

        // Sample pickup and scoring sequence
        MOVE_TO_SAMPLE_1(3),
        SAMPLE_1_OPERATIONS(4),
        MOVE_TO_SAMPLE_2(5),
        SAMPLE_2_OPERATIONS(6),
        MOVE_TO_SAMPLE_3(7),
        SAMPLE_3_OPERATIONS(8),

        // Specimen pickup and scoring cycles
        MOVE_TO_SPECIMEN_1(9),
        SPECIMEN_1_INTAKE(10),
        MOVE_TO_SCORE_2(11),
        SCORE_2_AND_INTAKE(12),
        MOVE_TO_SPECIMEN_2(13),
        SPECIMEN_2_INTAKE(14),
        MOVE_TO_SCORE_3(15),
        SCORE_3_AND_INTAKE(16),
        MOVE_TO_SPECIMEN_3(17),
        SPECIMEN_3_INTAKE(18),
        MOVE_TO_SCORE_4(19),
        SCORE_4_AND_INTAKE(20),
        MOVE_TO_SPECIMEN_4(21),
        SPECIMEN_4_INTAKE(22),
        MOVE_TO_SCORE_5(23),
        SCORE_5_AND_INTAKE(24),

        // Parking
        MOVE_TO_PARK(25),
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

        // Build individual path segments instead of using GeneratedPath
        buildIndividualPathSegments();

        // Set starting pose
        Pose startPose = new Pose(9.000, 63.000, Math.toRadians(0));
        follower.setStartingPose(startPose);

        // Initialize state
        pathState = AutoState.MOVE_TO_SCORE_1.getValue();
        commandExecuted = false;

        // Don't start preload scoring here - wait for start()

        telemetry.addData("Status", "Initialized - 13 Line Auto");
        telemetry.addData("Path State", pathState);
        telemetry.update();
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

    /**
     * Start raising vertical extension immediately to save time
     */
    private void startPreloadScoring() {
        SequentialCommandGroup preloadCommand = new SequentialCommandGroup(
            new InstantCommand(outtakeArm::close),
            new InstantCommand(() -> verticalExtension.setTargetPosition(VerticalExtension.OUTTAKE_POSITION))
        );
        CommandScheduler.getInstance().schedule(preloadCommand);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();

        // Start first path segment AND execute outtake command simultaneously
        follower.followPath(pathToScore1, true);
        outtakeCommand = new OuttakeCommand(verticalExtension, outtakeArm);
        CommandScheduler.getInstance().schedule(outtakeCommand);
        setPathState(AutoState.SCORE_PRELOAD.getValue());
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
            case 0: // MOVE_TO_SCORE_1 - Skip, already started
                setPathState(AutoState.SCORE_PRELOAD.getValue());
                break;

            case 1: // SCORE_PRELOAD
                if (!follower.isBusy() && verticalExtension.atTargetPosition()) {
                    if (!commandExecuted) {
                        // Just a brief pause since outtake arm is already in scoring position
                        currentCommand = new SequentialCommandGroup(
                            new WaitCommand(100) // Brief pause for scoring
                        );
                        CommandScheduler.getInstance().schedule(currentCommand);
                        commandExecuted = true;
                    }

                    if (commandExecuted && currentCommand.isFinished()) {
                        outtakeCommand = null; // Clear the reference
                        setPathState(AutoState.RETRACT_AFTER_PRELOAD.getValue());
                    }
                }
                break;

            case 2: // RETRACT_AFTER_PRELOAD
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted) {
                    follower.followPath(pathToSample1, true);
                    setPathState(AutoState.MOVE_TO_SAMPLE_1.getValue());
                }
                break;

            case 3: // MOVE_TO_SAMPLE_1
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_1_OPERATIONS.getValue());
                }
                break;

            case 4: // SAMPLE_1_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSample2, true);
                    setPathState(AutoState.MOVE_TO_SAMPLE_2.getValue());
                }
                break;

            case 5: // MOVE_TO_SAMPLE_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_2_OPERATIONS.getValue());
                }
                break;

            case 6: // SAMPLE_2_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSample3, true);
                    setPathState(AutoState.MOVE_TO_SAMPLE_3.getValue());
                }
                break;

            case 7: // MOVE_TO_SAMPLE_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_3_OPERATIONS.getValue());
                }
                break;

            case 8: // SAMPLE_3_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSpecimen1, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_1.getValue());
                }
                break;

            case 9: // MOVE_TO_SPECIMEN_1
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_1_INTAKE.getValue());
                }
                break;

            case 10: // SPECIMEN_1_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore2, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_2.getValue());
                }
                break;

            case 11: // MOVE_TO_SCORE_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_2_AND_INTAKE.getValue());
                }
                break;

            case 12: // SCORE_2_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(100), // Brief pause for scoring
                        new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSpecimen2, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_2.getValue());
                }
                break;

            case 13: // MOVE_TO_SPECIMEN_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_2_INTAKE.getValue());
                }
                break;

            case 14: // SPECIMEN_2_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore3, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_3.getValue());
                }
                break;

            case 15: // MOVE_TO_SCORE_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_3_AND_INTAKE.getValue());
                }
                break;

            case 16: // SCORE_3_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(100), // Brief pause for scoring
                        new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSpecimen3, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_3.getValue());
                }
                break;

            case 17: // MOVE_TO_SPECIMEN_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_3_INTAKE.getValue());
                }
                break;

            case 18: // SPECIMEN_3_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore4, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_4.getValue());
                }
                break;

            case 19: // MOVE_TO_SCORE_4
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_4_AND_INTAKE.getValue());
                }
                break;

            case 20: // SCORE_4_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(100), // Brief pause for scoring
                        new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToSpecimen4, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_4.getValue());
                }
                break;

            case 21: // MOVE_TO_SPECIMEN_4
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_4_INTAKE.getValue());
                }
                break;

            case 22: // SPECIMEN_4_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore5, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_5.getValue());
                }
                break;

            case 23: // MOVE_TO_SCORE_5
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_5_AND_INTAKE.getValue());
                }
                break;

            case 24: // SCORE_5_AND_INTAKE
                if (!commandExecuted) {
                    // Just a brief pause since outtake arm is already in scoring position - no intake after final score
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(300) // Brief pause for final scoring
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToPark, true);
                    setPathState(AutoState.MOVE_TO_PARK.getValue());
                }
                break;

            case 25: // MOVE_TO_PARK
                if (!follower.isBusy()) {
                    setPathState(AutoState.COMPLETE.getValue());
                }
                break;

            case -1: // COMPLETE
                // Autonomous complete
                break;

            default:
                setPathState(AutoState.COMPLETE.getValue());
                break;
        }
    }

    /**
     * Creates sample operations: extend sliders, pickup, retract, drop
     */
    private SequentialCommandGroup createSampleOperationsCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> horizontalSlider.setTargetPosition(HORIZONTAL_EXTEND_POSITION)),
            new WaitCommand(800), // Wait for extension
            new PickupCommand(gripper, arm),
            new WaitCommand(500), // Wait for pickup
            new InstantCommand(() -> horizontalSlider.retract()),
            new WaitCommand(800), // Wait for retraction
            new DropCommand(gripper, arm)
        );
    }

    /**
     * Creates score and intake command for specimens while backing away
     */
    private SequentialCommandGroup createScoreAndIntakeCommand() {
        return new SequentialCommandGroup(
            new OuttakeCommand(verticalExtension, outtakeArm),
            new WaitCommand(500), // Wait for scoring
            new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
        );
    }

    /**
     * Sets the path state and resets necessary flags and timers
     * This is crucial for proper state machine operation
     */
    public void setPathState(int pState) {
        pathState = pState;
        commandExecuted = false; // Reset the command execution flag
        pathTimer.resetTimer();
    }

    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("Command Executed", commandExecuted);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Runtime", String.format("%.1f / 30.0s", opmodeTimer.getElapsedTimeSeconds()));

        // Progress indicator
        double progress = (pathState + 1.0) / 26.0 * 100.0;
        telemetry.addData("Progress", String.format("%.1f%%", progress));

        // Subsystem status
        telemetry.addData("Horizontal Pos", horizontalSlider.getCurrentPosition());
        telemetry.addData("Vertical Pos", verticalExtension.getCurrentPosition());

        // Robot pose
        Pose currentPose = follower.getPose();
        telemetry.addData("X", String.format("%.1f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.1f", currentPose.getY()));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all subsystems
        CommandScheduler.getInstance().reset();
        horizontalSlider.stop();
        verticalExtension.stop();

        // Stop follower
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    /**
     * Build individual path segments based on GeneratedPath coordinates
     * This approach allows precise stops at each position
     */
    private void buildIndividualPathSegments() {
        // Define all poses from the GeneratedPath coordinates
        Pose startPose = new Pose(9.000, 63.000, Math.toRadians(0));
        Pose scorePose1 = new Pose(38.000, 63.000, Math.toRadians(0));  // Line 1 end
        Pose samplePose1 = new Pose(16.000, 22.000, Math.toRadians(0)); // Line 2 end
        Pose samplePose2 = new Pose(16.000, 13.000, Math.toRadians(0)); // Line 3 end
        Pose samplePose3 = new Pose(16.000, 10.000, Math.toRadians(-14)); // Line 4 end
        Pose specimenPose1 = new Pose(8.000, 33.000, Math.toRadians(0)); // Line 5 end
        Pose scorePose2 = new Pose(38.000, 63.000, Math.toRadians(0));  // Line 6 end
        Pose specimenPose2 = new Pose(8.000, 33.000, Math.toRadians(0)); // Line 7 end
        Pose scorePose3 = new Pose(38.000, 63.000, Math.toRadians(0));  // Line 8 end
        Pose specimenPose3 = new Pose(8.000, 33.000, Math.toRadians(0)); // Line 9 end
        Pose scorePose4 = new Pose(38.000, 63.000, Math.toRadians(0));  // Line 10 end
        Pose specimenPose4 = new Pose(8.000, 33.000, Math.toRadians(0)); // Line 11 end
        Pose scorePose5 = new Pose(38.000, 63.000, Math.toRadians(0));  // Line 12 end
        Pose parkPose = new Pose(8.000, 29.000, Math.toRadians(0));     // Line 13 end

        // Build individual path segments
        // Line 1: Move to scoring position 1
        pathToScore1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(startPose),
                        new com.pedropathing.pathgen.Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 2: Move to sample pickup 1
        pathToSample1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose1),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(samplePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 3: Move to sample pickup 2
        pathToSample2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(samplePose1),
                        new com.pedropathing.pathgen.Point(samplePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 4: Move to sample pickup 3
        pathToSample3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(samplePose2),
                        new com.pedropathing.pathgen.Point(samplePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-14))
                .build();

        // Line 5: Move to specimen pickup 1
        pathToSpecimen1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(samplePose3),
                        new com.pedropathing.pathgen.Point(specimenPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 6: Move to scoring 2
        pathToScore2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(specimenPose1),
                        new com.pedropathing.pathgen.Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 7: Move to specimen pickup 2
        pathToSpecimen2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose2),
                        new com.pedropathing.pathgen.Point(specimenPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 8: Move to scoring 3
        pathToScore3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(specimenPose2),
                        new com.pedropathing.pathgen.Point(scorePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 9: Move to specimen pickup 3
        pathToSpecimen3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose3),
                        new com.pedropathing.pathgen.Point(specimenPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 10: Move to scoring 4
        pathToScore4 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(specimenPose3),
                        new com.pedropathing.pathgen.Point(scorePose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 11: Move to specimen pickup 4
        pathToSpecimen4 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose4),
                        new com.pedropathing.pathgen.Point(specimenPose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 12: Move to scoring 5
        pathToScore5 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(specimenPose4),
                        new com.pedropathing.pathgen.Point(scorePose5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 13: Move to parking
        pathToPark = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose5),
                        new com.pedropathing.pathgen.Point(parkPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
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
@Config
@Autonomous(name = "Full Competition Auto", group = "Main")
public class FullCompetitionAuto extends OpMode {

    // Pedro Pathing components
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Individual path segments (consolidated for efficiency)
    private PathChain pathToScore1, pathToAfter1, pathToSample1, pathToSample2, pathToSample3;
    private PathChain pathToSpecimen1, pathToScore2, pathToSpecimen2, pathToScore3;
    private PathChain pathToSpecimen3, pathToScore4, pathToSpecimen4, pathToScore5, pathToPark;

    private PathChain pathToAfter2, pathToAfter3, pathToAfter4, pathToAfter5;

    public static double timer = 0, timer2=0, timer3=0, timer4=0, timer5=0;

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
    private OuttakeCommand outtakeCommand;
    private IntakeSpecimenCommand intakeCommand;

    // Constants
    private static final double HORIZONTAL_EXTEND_POSITION = 35000;
    private static final double ACTION_TIMEOUT = 2.5; // Reduced for speed
    private static final int TOTAL_SEGMENTS = 13;

    // Auto states - one per path segment
    private enum AutoState {
        // Preloaded specimen scoring
        MOVE_TO_SCORE_1(0),
        SCORE_PRELOAD(1),
        MOVE_TO_AFTER_1(2),
        RETRACT_AFTER_PRELOAD(3),

        // Sample pickup and scoring sequence
        MOVE_TO_SAMPLE_1(4),
        SAMPLE_1_OPERATIONS(5),
        MOVE_TO_SAMPLE_2(6),
        SAMPLE_2_OPERATIONS(7),
        MOVE_TO_SAMPLE_3(8),
        SAMPLE_3_OPERATIONS(9),

        // Specimen pickup and scoring cycles
        MOVE_TO_SPECIMEN_1(10),
        SPECIMEN_1_INTAKE(11),
        MOVE_TO_SCORE_2(12),
        SCORE_2_AND_INTAKE(13),
        MOVE_TO_AFTER_2(14),
        MOVE_TO_SPECIMEN_2(15),
        SPECIMEN_2_INTAKE(16),
        MOVE_TO_SCORE_3(17),
        SCORE_3_AND_INTAKE(18),
        MOVE_TO_AFTER_3(19),
        MOVE_TO_SPECIMEN_3(20),
        SPECIMEN_3_INTAKE(21),
        MOVE_TO_SCORE_4(22),
        SCORE_4_AND_INTAKE(23),
        MOVE_TO_AFTER_4(24),
        MOVE_TO_SPECIMEN_4(25),
        SPECIMEN_4_INTAKE(26),
        MOVE_TO_SCORE_5(27),
        SCORE_5_AND_INTAKE(28),
        MOVE_TO_AFTER_5(29),

        // Parking
        MOVE_TO_PARK(30),
        COMPLETE(-1);

        private final int value;
        AutoState(int value) { this.value = value; }
        public int getValue() { return value; }
    }

    @Override
    public void init() {
        // Initialize Pedro Pathing
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

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

        // Set initial positions
        outtakeArm.goToInit();
        arm.goToInit();

        outtakeArm.close();
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

        setPathState(0);
        arm.set(Arm.PIVOT_INITIAL, Arm.ROTATION_INITIAL);
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
            case 0:
                if(actionTimer.getElapsedTimeSeconds() > 0.01) {
                    outtakeCommand = new OuttakeCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(outtakeCommand);
                    follower.followPath(pathToScore1, true);
                    setPathState(1);
                }
                break;

            case 1: // SCORE_PRELOAD

                if(actionTimer.getElapsedTimeSeconds() > timer){
                    outtakeArm.open();
                }

                if(actionTimer.getElapsedTimeSeconds() > timer + 0.1){
                    follower.followPath(pathToSample1);
                    setPathState(2);
                }
                break;

            case 2: // MOVE_TO_AFTER_1
                if(actionTimer.getElapsedTimeSeconds() > 0.1){
                    outtakeArm.goToIntake();
                    setPathState(3);
                }
                break;

            case 3: // RETRACT_AFTER_PRELOAD
                if(actionTimer.getElapsedTimeSeconds() > timer2){
                    setPathState(99999);
                }

                break;

            case 4: // MOVE_TO_SAMPLE_1
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_1_OPERATIONS.getValue());
                }
                break;

            case 5: // SAMPLE_1_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                // Add timeout mechanism for stuck sample operations
                boolean sampleFinished = currentCommand != null && currentCommand.isFinished();
                boolean sampleTimeout = actionTimer.getElapsedTimeSeconds() > 2;

                if (commandExecuted && (sampleFinished || sampleTimeout)) {
                    if (sampleTimeout && !sampleFinished) {
                        telemetry.addData("⚠️ TIMEOUT", "Proceeding without sample operations completion");
                        telemetry.update();
                    }

                    follower.followPath(pathToSample2, true);
                    setPathState(AutoState.MOVE_TO_SAMPLE_2.getValue());
                }
                break;

            case 6: // MOVE_TO_SAMPLE_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_2_OPERATIONS.getValue());
                }
                break;

            case 7: // SAMPLE_2_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                // Add timeout mechanism for stuck sample operations
                boolean sample2Finished = currentCommand != null && currentCommand.isFinished();
                boolean sample2Timeout = actionTimer.getElapsedTimeSeconds() > 2;

                if (commandExecuted && (sample2Finished || sample2Timeout)) {
                    if (sample2Timeout && !sample2Finished) {
                        telemetry.addData("⚠️ TIMEOUT", "Proceeding without sample 2 operations completion");
                        telemetry.update();
                    }

                    follower.followPath(pathToSample3, true);
                    setPathState(AutoState.MOVE_TO_SAMPLE_3.getValue());
                }
                break;

            case 8: // MOVE_TO_SAMPLE_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SAMPLE_3_OPERATIONS.getValue());
                }
                break;

            case 9: // SAMPLE_3_OPERATIONS
                if (!commandExecuted) {
                    currentCommand = createSampleOperationsCommand();
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                // Add timeout mechanism for stuck sample operations
                boolean sample3Finished = currentCommand != null && currentCommand.isFinished();
                boolean sample3Timeout = actionTimer.getElapsedTimeSeconds() > 2;

                if (commandExecuted && (sample3Finished || sample3Timeout)) {
                    if (sample3Timeout && !sample3Finished) {
                        telemetry.addData("⚠️ TIMEOUT", "Proceeding without sample 3 operations completion");
                        telemetry.update();
                    }

                    follower.followPath(pathToSpecimen1, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_1.getValue());
                }
                break;

            case 10: // MOVE_TO_SPECIMEN_1
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_1_INTAKE.getValue());
                }
                break;

            case 11: // SPECIMEN_1_INTAKE
                if (!commandExecuted) {
                    arm.goToInit();
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                // Add timeout mechanism for stuck specimen intake
                boolean specimen1Finished = currentCommand != null && currentCommand.isFinished();
                boolean specimen1Timeout = actionTimer.getElapsedTimeSeconds() > 0.6;

                if (commandExecuted && (specimen1Finished || specimen1Timeout)) {
                    if (specimen1Timeout && !specimen1Finished) {
                        telemetry.addData("⚠️ TIMEOUT", "Proceeding without specimen 1 intake completion");
                        telemetry.update();
                    }

                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore2, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_2.getValue());
                }
                break;

            case 12: // MOVE_TO_SCORE_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_2_AND_INTAKE.getValue());
                }
                break;

            case 13: // SCORE_2_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                // Add timeout mechanism for stuck score and intake
                boolean score2Finished = currentCommand != null && currentCommand.isFinished();
                boolean score2Timeout = actionTimer.getElapsedTimeSeconds() > 0.1;

                if (commandExecuted && (score2Finished || score2Timeout)) {
                    if (score2Timeout && !score2Finished) {
                        telemetry.addData("⚠️ TIMEOUT", "Proceeding without score 2 completion");
                        telemetry.update();
                    }

                    setPathState(AutoState.MOVE_TO_AFTER_2.getValue());
                    outtakeArm.open();
                    follower.followPath(pathToAfter2);
                }
                break;

            case 14: // MOVE_TO_AFTER_2
                if (!follower.isBusy()) {
                    follower.followPath(pathToSpecimen2, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_2.getValue());
                }
                break;

            case 15: // MOVE_TO_SPECIMEN_2
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_2_INTAKE.getValue());
                }
                break;

            case 16: // SPECIMEN_2_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore3, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_3.getValue());
                }
                break;

            case 17: // MOVE_TO_SCORE_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_3_AND_INTAKE.getValue());
                }
                break;

            case 18: // SCORE_3_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(100), // Brief pause for scoring
                        new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    outtakeArm.open();
                    follower.followPath(pathToAfter2);
                    setPathState(AutoState.MOVE_TO_AFTER_3.getValue());
                }
                break;

            case 19: // MOVE_TO_AFTER_3
                if (!follower.isBusy()) {
                    follower.followPath(pathToSpecimen3, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_3.getValue());
                }
                break;

            case 20: // MOVE_TO_SPECIMEN_3
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_3_INTAKE.getValue());
                }
                break;

            case 21: // SPECIMEN_3_INTAKE
                if (!commandExecuted) {
                    currentCommand = new IntakeSpecimenCommand(verticalExtension, outtakeArm);
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    // Start moving AND execute outtake command simultaneously
                    follower.followPath(pathToScore4, true);
                    CommandScheduler.getInstance().schedule(new OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(AutoState.MOVE_TO_SCORE_4.getValue());
                }
                break;

            case 22: // MOVE_TO_SCORE_4
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_4_AND_INTAKE.getValue());
                }
                break;

            case 23: // SCORE_4_AND_INTAKE
                if (!commandExecuted) {
                    // Just execute intake since outtake arm is already in scoring position
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(100), // Brief pause for scoring
                        new IntakeSpecimenCommand(verticalExtension, outtakeArm) // Intake while backing away
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(pathToAfter4, true);
                    setPathState(AutoState.MOVE_TO_AFTER_4.getValue());
                }
                break;

            case 24: // MOVE_TO_AFTER_4
                if (!follower.isBusy()) {
                    follower.followPath(pathToSpecimen4, true);
                    setPathState(AutoState.MOVE_TO_SPECIMEN_4.getValue());
                }
                break;

            case 25: // MOVE_TO_SPECIMEN_4
                if (!follower.isBusy()) {
                    setPathState(AutoState.SPECIMEN_4_INTAKE.getValue());
                }
                break;

            case 26: // SPECIMEN_4_INTAKE
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

            case 27: // MOVE_TO_SCORE_5
                if (!follower.isBusy()) {
                    setPathState(AutoState.SCORE_5_AND_INTAKE.getValue());
                }
                break;

            case 28: // SCORE_5_AND_INTAKE
                if (!commandExecuted) {
                    // Just a brief pause since outtake arm is already in scoring position - no intake after final score
                    currentCommand = new SequentialCommandGroup(
                        new WaitCommand(300) // Brief pause for final scoring
                    );
                    CommandScheduler.getInstance().schedule(currentCommand);
                    commandExecuted = true;
                }

                if (commandExecuted && currentCommand.isFinished()) {
                    follower.followPath(pathToAfter5, true);
                    setPathState(AutoState.MOVE_TO_AFTER_5.getValue());
                }
                break;

            case 29: // MOVE_TO_AFTER_5
                if (!follower.isBusy()) {
                    follower.followPath(pathToPark, true);
                    setPathState(AutoState.MOVE_TO_PARK.getValue());
                }
                break;

            case 30: // MOVE_TO_PARK
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
     * Sets the current path state and resets command execution tracking.
     * @param newState The new state to set.
     */
    private void setPathState(int newState) {
        pathState = newState;
        commandExecuted = false;
        currentCommand = null; // Clear previous command reference
        actionTimer.resetTimer(); // Reset action timer for the new state
    }

    /**
     * Creates sample operations: extend sliders, pickup, retract, drop
     */
    private SequentialCommandGroup createSampleOperationsCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> horizontalSlider.setTargetPosition(HORIZONTAL_EXTEND_POSITION)),
            new WaitCommand(600), // Wait for extension
            new PickupCommand(gripper, arm),
            new WaitCommand(300), // Wait for pickup
            new InstantCommand(() -> horizontalSlider.retract()),
            new WaitCommand(600), // Wait for retraction
            new DropCommand(gripper, arm),
            new WaitCommand(300)
        );
    }


    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        // === PRIMARY STATUS ===
        telemetry.addData("========== AUTO STATUS ==========", "");
        telemetry.addData("Path State", pathState + " (" + getStateDescription(pathState) + ")");
        telemetry.addData("Command Executed", commandExecuted);
        telemetry.addData("Runtime", String.format(java.util.Locale.US, "%.1f / 30.0s", opmodeTimer.getElapsedTimeSeconds()));

        // Progress indicator
        double progress = (pathState + 1.0) / 26.0 * 100.0;
        telemetry.addData("Progress", String.format(java.util.Locale.US, "%.1f%%", progress));

        // === PATH FOLLOWING STATUS ===
        telemetry.addData("========== PATH STATUS ==========", "");
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Action Timer", String.format(java.util.Locale.US, "%.2fs", actionTimer.getElapsedTimeSeconds()));

        // Robot pose
        Pose currentPose = follower.getPose();
        telemetry.addData("Robot X", String.format(java.util.Locale.US, "%.1f", currentPose.getX()));
        telemetry.addData("Robot Y", String.format(java.util.Locale.US, "%.1f", currentPose.getY()));
        telemetry.addData("Robot Heading", String.format(java.util.Locale.US, "%.1f°", Math.toDegrees(currentPose.getHeading())));

        // === COMMAND STATUS ===
        telemetry.addData("========== COMMAND STATUS ==========", "");
        if (currentCommand != null) {
            telemetry.addData("Current Command", currentCommand.getClass().getSimpleName());
            telemetry.addData("Command Finished", currentCommand.isFinished());
            telemetry.addData("Command Scheduled", currentCommand.isScheduled());
        } else {
            telemetry.addData("Current Command", "None");
        }

        if (outtakeCommand != null) {
            telemetry.addData("Outtake Command", "Active");
            telemetry.addData("Outtake Finished", outtakeCommand.isFinished());
        } else {
            telemetry.addData("Outtake Command", "None");
        }

        // === SUBSYSTEM STATUS ===
        telemetry.addData("========== SUBSYSTEMS ==========", "");
        telemetry.addData("Horizontal Pos", horizontalSlider.getCurrentPosition());
        telemetry.addData("Horizontal Target", horizontalSlider.getTargetPosition());
        telemetry.addData("Horizontal At Target", horizontalSlider.atTargetPosition());

        telemetry.addData("Vertical Pos", verticalExtension.getCurrentPosition());
        telemetry.addData("Vertical Target", verticalExtension.getTargetPosition());
        telemetry.addData("Vertical At Target", verticalExtension.atTargetPosition());

        // Note: Using generic status since exact method names may vary
        telemetry.addData("Outtake Arm State", "Active"); // Replace with actual state method
        telemetry.addData("Gripper State", "Active"); // Replace with actual state method

        // === DEBUG INFO FOR CURRENT STATE ===
        telemetry.addData("========== DEBUG INFO ==========", "");
        addStateSpecificDebug();

        telemetry.update();
    }

    /**
     * Get human-readable description of current state
     */
    private String getStateDescription(int state) {
        switch (state) {
            case 0: return "MOVE_TO_SCORE_1";
            case 1: return "SCORE_PRELOAD";
            case 2: return "MOVE_TO_AFTER_1";
            case 3: return "RETRACT_AFTER_PRELOAD";
            case 4: return "MOVE_TO_SAMPLE_1";
            case 5: return "SAMPLE_1_OPERATIONS";
            case 6: return "MOVE_TO_SAMPLE_2";
            case 7: return "SAMPLE_2_OPERATIONS";
            case 8: return "MOVE_TO_SAMPLE_3";
            case 9: return "SAMPLE_3_OPERATIONS";
            case 10: return "MOVE_TO_SPECIMEN_1";
            case 11: return "SPECIMEN_1_INTAKE";
            case 12: return "MOVE_TO_SCORE_2";
            case 13: return "SCORE_2_AND_INTAKE";
            case 14: return "MOVE_TO_AFTER_2";
            case 15: return "MOVE_TO_SPECIMEN_2";
            case 16: return "SPECIMEN_2_INTAKE";
            case 17: return "MOVE_TO_SCORE_3";
            case 18: return "SCORE_3_AND_INTAKE";
            case 19: return "MOVE_TO_AFTER_3";
            case 20: return "MOVE_TO_SPECIMEN_3";
            case 21: return "SPECIMEN_3_INTAKE";
            case 22: return "MOVE_TO_SCORE_4";
            case 23: return "SCORE_4_AND_INTAKE";
            case 24: return "MOVE_TO_AFTER_4";
            case 25: return "MOVE_TO_SPECIMEN_4";
            case 26: return "SPECIMEN_4_INTAKE";
            case 27: return "MOVE_TO_SCORE_5";
            case 28: return "SCORE_5_AND_INTAKE";
            case 29: return "MOVE_TO_AFTER_5";
            case 30: return "MOVE_TO_PARK";
            case -1: return "COMPLETE";
            default: return "UNKNOWN";
        }
    }

    /**
     * Add debug information specific to current state
     */
    private void addStateSpecificDebug() {
        switch (pathState) {
            case 1: // SCORE_PRELOAD
                telemetry.addData("Waiting for", "Path complete AND vertical at target");
                telemetry.addData("Path Complete", !follower.isBusy());
                telemetry.addData("Vertical Ready", verticalExtension.atTargetPosition());

                // Additional vertical extension debugging
                double verticalPos = verticalExtension.getCurrentPosition();
                double verticalTarget = verticalExtension.getTargetPosition();
                double verticalError = Math.abs(verticalTarget - verticalPos);
                telemetry.addData("Vertical Error", String.format(java.util.Locale.US, "%.0f counts", verticalError));
                telemetry.addData("Vertical Power", "Check motor power"); // Add actual power if available

                // Timeout warning
                if (actionTimer.getElapsedTimeSeconds() > 5.0) {
                    telemetry.addData("⚠️ WARNING", "Vertical extension timeout!");
                    telemetry.addData("Suggestion", "Check mechanical obstruction");
                }
                break;

            case 2: // MOVE_TO_AFTER_1
                telemetry.addData("Following path", "to AFTER scoring position 1");
                break;

            case 3: // RETRACT_AFTER_PRELOAD
                telemetry.addData("Waiting for", "Intake command to finish");
                if (currentCommand != null) {
                    telemetry.addData("Command Type", "IntakeSpecimenCommand");
                }

                // Timeout check
                if (actionTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addData("⚠️ WARNING", "Intake command timeout!");
                }
                break;

            case 4: case 6: case 8: // SAMPLE_OPERATIONS
                telemetry.addData("Waiting for", "Sample operations to complete");
                telemetry.addData("Horizontal Extended", horizontalSlider.getCurrentPosition() > 30000);

                // Show which part of sample operations we're in
                if (currentCommand != null) {
                    telemetry.addData("Operation Stage", "Check command progress");
                }

                // Timeout check
                if (actionTimer.getElapsedTimeSeconds() > 10.0) {
                    telemetry.addData("⚠️ WARNING", "Sample operations timeout!");
                }
                break;

            case 10: case 14: case 18: case 22: // SPECIMEN_INTAKE
                telemetry.addData("Waiting for", "Specimen intake to complete");

                // Timeout check
                if (actionTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addData("⚠️ WARNING", "Specimen intake timeout!");
                }
                break;

            case 12: case 16: case 20: case 24: // SCORE_AND_INTAKE
                telemetry.addData("Waiting for", "Score + Intake sequence");

                // Timeout check
                if (actionTimer.getElapsedTimeSeconds() > 5.0) {
                    telemetry.addData("⚠️ WARNING", "Score+Intake sequence timeout!");
                }
                break;

            case 5: case 7: case 9: case 11: case 13:
            case 15: case 17: case 19: case 21: case 23: case 25: // MOVE states
                telemetry.addData("Waiting for", "Path following to complete");
                telemetry.addData("Path Distance", String.format(java.util.Locale.US, "%.1f", getDistanceToTarget()));

                // Timeout check for path following
                if (actionTimer.getElapsedTimeSeconds() > 8.0) {
                    telemetry.addData("⚠️ WARNING", "Path following timeout!");
                    telemetry.addData("Suggestion", "Check path or robot position");
                }
                break;

            case -1: // COMPLETE
                telemetry.addData("Status", "AUTONOMOUS COMPLETE!");
                break;

            default:
                telemetry.addData("Unknown State", pathState);
                break;
        }

        // General timeout and safety checks
        if (opmodeTimer.getElapsedTimeSeconds() > 28.0) {
            telemetry.addData("���� CRITICAL", "Near 30-second limit!");
        }
    }

    /**
     * Calculate approximate distance to current path target (simplified)
     */
    private double getDistanceToTarget() {
        // This is a simplified calculation - in a real implementation you'd want
        // to get the actual target pose from the current path
        return follower.isBusy() ? 999.0 : 0.0;
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

    private void buildIndividualPathSegments() {
        // Define all poses from the GeneratedPath coordinates
        Pose startPose = new Pose(9.000, 63.000, Math.toRadians(0));
        Pose scorePose1 = new Pose(45.000, 63.000, Math.toRadians(0));  // Line 1 end
        Pose afterScorePose = new Pose(39.000, 63.000, Math.toRadians(0));
        Pose samplePose1 = new Pose(14.000, 20.000, Math.toRadians(0)); // Line 2 end
        Pose samplePose2 = new Pose(14.000, 9.000, Math.toRadians(0)); // Line 3 end
        Pose samplePose3 = new Pose(19.000, 7.500, Math.toRadians(-20)); // Line 4 end
        Pose specimenPose1 = new Pose(11.000, 10.000, Math.toRadians(0)); // Line 5 end
        Pose scorePose2 = new Pose(43.000, 63.000, Math.toRadians(0));  // Line 6 end
        Pose specimenPose2 = new Pose(11.000, 30.000, Math.toRadians(0)); // Line 7 end
        Pose scorePose3 = new Pose(43.000, 63.000, Math.toRadians(0));  // Line 8 end
        Pose specimenPose3 = new Pose(11.000, 30.000, Math.toRadians(0)); // Line 9 end
        Pose scorePose4 = new Pose(43.000, 63.000, Math.toRadians(0));  // Line 10 end
        Pose specimenPose4 = new Pose(11.000, 30.000, Math.toRadians(0)); // Line 11 end
        Pose scorePose5 = new Pose(43.000, 63.000, Math.toRadians(0));  // Line 12 end
        Pose parkPose = new Pose(11.000, 34.000, Math.toRadians(0));     // Line 13 end

        // Build individual path segments
        // Line 1: Move to scoring position 1
        pathToScore1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(startPose),
                        new com.pedropathing.pathgen.Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToAfter1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose1),
                        new com.pedropathing.pathgen.Point(afterScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 2: Move to sample pickup 1 (starts from afterScorePose, not scorePose1)
        pathToSample1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose1),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(samplePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 3: Move to sample pickup 2
        pathToSample2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(samplePose1),
                        new com.pedropathing.pathgen.Point(5.000, 17.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(samplePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 4: Move to sample pickup 3
        pathToSample3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(samplePose2),
                        new com.pedropathing.pathgen.Point(5.000, 7.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(samplePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // Line 5: Move to specimen pickup 1
        pathToSpecimen1 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(samplePose3),
                        new com.pedropathing.pathgen.Point(specimenPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 6: Move to scoring position 2
        pathToScore2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(specimenPose1),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToAfter2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose2),
                        new com.pedropathing.pathgen.Point(afterScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 7: Move to specimen pickup 2
        pathToSpecimen2 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose2),
                        new com.pedropathing.pathgen.Point(26.000, 35.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(specimenPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 8: Move to scoring position 3
        pathToScore3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(specimenPose2),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(scorePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToAfter3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose3),
                        new com.pedropathing.pathgen.Point(afterScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 9: Move to specimen pickup 3
        pathToSpecimen3 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose3),
                        new com.pedropathing.pathgen.Point(26.000, 35.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(specimenPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 10: Move to scoring position 4
        pathToScore4 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(specimenPose3),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(scorePose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToAfter4 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose4),
                        new com.pedropathing.pathgen.Point(afterScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 11: Move to specimen pickup 4
        pathToSpecimen4 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(scorePose4),
                        new com.pedropathing.pathgen.Point(26.000, 35.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(specimenPose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 12: Move to scoring position 5
        pathToScore5 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierCurve(
                        new com.pedropathing.pathgen.Point(specimenPose4),
                        new com.pedropathing.pathgen.Point(13.000, 52.000, com.pedropathing.pathgen.Point.CARTESIAN),
                        new com.pedropathing.pathgen.Point(scorePose5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathToAfter5 = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose5),
                        new com.pedropathing.pathgen.Point(afterScorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Line 13: Move to parking position
        pathToPark = follower.pathBuilder()
                .addPath(new com.pedropathing.pathgen.BezierLine(
                        new com.pedropathing.pathgen.Point(scorePose5),
                        new com.pedropathing.pathgen.Point(parkPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}

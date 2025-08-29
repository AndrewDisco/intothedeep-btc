package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoModule;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlider;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.subsystems.VerticalExtension;
import org.opencv.core.Mat;

@Config
@Autonomous
public class traiect7spec extends OpMode {


    // Subsystems
    private HorizontalSlider horizontalSlider;
    private VerticalExtension verticalExtension;
    private OuttakeArm outtakeArm;
    private Gripper gripper;
    private Arm arm;
    private ExtendoModule extendo;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose firstScore = new Pose(32, 13, Math.toRadians(0));
    private final Pose firstGrab = new Pose(29, 16, Math.toRadians(0));
    private final Pose firstWall = new Pose(1.5, -22, Math.toRadians(0));
    private final Pose controlPose1 = new Pose(10, -15, Math.toRadians(0));
    private final Pose sfarsitPrimaCurba = new Pose(39, -19.5, Math.toRadians(0));
    private final Pose controlPose3 = new Pose(45, -18, Math.toRadians(0));
    private final Pose firstPush = new Pose(49, -31.5, Math.toRadians(0));
    private final Pose firstPushEnd = new Pose(14, -31.5, Math.toRadians(0));
    private final Pose secondPush = new Pose(49, -40, Math.toRadians(0));
    private final Pose controlPose4 = new Pose(45, -26, Math.toRadians(0));
    private final Pose secondPushEnd = new Pose(14, -40, Math.toRadians(0));
    private final Pose thirdPush = new Pose(48, -45.5, Math.toRadians(0));
    private final Pose controlPose5 = new Pose(45, -40, Math.toRadians(0));
    private final Pose thirdPushEnd = new Pose(1.1, -45.5, Math.toRadians(0));
    private MotorEx sliderMotor;

    public static double timer1=0, timer2=0, timer3=0, timer4=0;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private Path goScoreFirst, goGrabFirst, goWallFirst, goScoreSecond, goPushFirstPrep, goPushFirst,
            goPushFirstEnd, goPushSecond, goPushSecondEnd, goPushThird, goPushThirdEnd,
            goScoreThird, goGrabSecond, goThirdWall, goScoreFourth, goScoreFifth;

    public void buildPaths() {
        goScoreFirst = new Path(new BezierLine(new Point(startPose), new Point(firstScore)));
        goScoreFirst.setConstantHeadingInterpolation(Math.toRadians(0));

        goGrabFirst = new Path(new BezierLine(new Point(firstScore), new Point(firstGrab)));
        goGrabFirst.setConstantHeadingInterpolation(Math.toRadians(0));

        goWallFirst = new Path(new BezierLine(new Point(firstScore), new Point(firstWall)));
        goWallFirst.setConstantHeadingInterpolation(Math.toRadians(0));

        goScoreSecond = new Path(new BezierLine(new Point(firstWall), new Point(firstScore)));
        goScoreSecond.setConstantHeadingInterpolation(Math.toRadians(15));

        goPushFirstPrep = new Path(new BezierCurve(new Point(firstScore), new Point(controlPose1), new Point(sfarsitPrimaCurba)));
        goPushFirstPrep.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushFirst = new Path(new BezierCurve(new Point(sfarsitPrimaCurba), new Point(controlPose3), new Point(firstPush)));
        goPushFirst.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushFirstEnd = new Path(new BezierLine(new Point(firstPush), new Point(firstPushEnd)));
        goPushFirstEnd.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushSecond = new Path(new BezierCurve(new Point(firstPushEnd), new Point(controlPose4), new Point(secondPush)));
        goPushSecond.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushSecondEnd = new Path(new BezierLine(new Point(secondPush), new Point(secondPushEnd)));
        goPushSecondEnd.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushThird = new Path(new BezierCurve(new Point(secondPushEnd), new Point(controlPose5), new Point(thirdPush)));
        goPushThird.setConstantHeadingInterpolation(Math.toRadians(0));

        goPushThirdEnd = new Path(new BezierLine(new Point(thirdPush), new Point(thirdPushEnd)));
        goPushThirdEnd.setConstantHeadingInterpolation(Math.toRadians(0));

        goScoreThird = new Path(new BezierLine(new Point(thirdPushEnd), new Point(firstScore)));
        goScoreThird.setConstantHeadingInterpolation(Math.toRadians(15));

        goThirdWall = new Path(new BezierLine(new Point(firstScore), new Point(firstWall)));
        goThirdWall.setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0));

        goScoreFourth = new Path(new BezierLine(new Point(firstWall), new Point(firstScore)));
        goScoreFourth.setConstantHeadingInterpolation(Math.toRadians(15));

        goScoreFifth = new Path(new BezierLine(new Point(firstWall), new Point(firstScore)));
        goScoreFifth.setConstantHeadingInterpolation(Math.toRadians(15));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if(pathTimer.getElapsedTimeSeconds() > 0) {
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.OuttakeCommand(verticalExtension, outtakeArm));
                    setPathState(0);
                }
                break;
            case 0:
                if(pathTimer.getElapsedTimeSeconds() > 0.01) {
                    //outtakeArm.goToOuttake();
                    follower.followPath(goScoreFirst);
                    setPathState(1);
                }
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    outtakeArm.open();
                    follower.followPath(goWallFirst);
                }

                if(pathTimer.getElapsedTimeSeconds() > 1.5){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand(verticalExtension, outtakeArm));
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.OuttakeCommand(verticalExtension, outtakeArm));
                    follower.followPath(goScoreSecond);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    outtakeArm.open();
                    follower.followPath(goPushFirstPrep);
                }

                if(pathTimer.getElapsedTimeSeconds() > 2.3){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand(verticalExtension, outtakeArm));
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(goPushFirst);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(goPushFirstEnd);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(goPushSecond);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(goPushSecondEnd);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(goPushThird);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(goPushThirdEnd);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.OuttakeCommand(verticalExtension, outtakeArm));
                    follower.followPath(goScoreThird);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(goGrabFirst);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    outtakeArm.open();
                    follower.followPath(goWallFirst);
                }

                if(pathTimer.getElapsedTimeSeconds() > 1){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand(verticalExtension, outtakeArm));
                    setPathState(13);
                }
                break;
            case 13:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.OuttakeCommand(verticalExtension, outtakeArm));
                    follower.followPath(goScoreFourth);
                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    outtakeArm.open();
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.3){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand(verticalExtension, outtakeArm));
                    follower.followPath(goWallFirst);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.OuttakeCommand(verticalExtension, outtakeArm));
                    follower.followPath(goScoreFifth);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 2){
                    outtakeArm.open();
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.3){
                    CommandScheduler.getInstance().schedule(new org.firstinspires.ftc.teamcode.commands.IntakeSpecimenCommand(verticalExtension, outtakeArm));
                    follower.followPath(goWallFirst);
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    sliderMotor.set(-0.5);
                    sliderMotor.resetEncoder();
                }

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();
        extendo.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("sliders target", verticalExtension.getTargetPosition());
        telemetry.addData("sliders pos", verticalExtension.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void init() {
        //horizontalSlider = new HorizontalSlider(hardwareMap);
        extendo = new ExtendoModule(hardwareMap);
        verticalExtension = new VerticalExtension(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        arm = new Arm(hardwareMap);
        sliderMotor = new MotorEx(hardwareMap, "glisiere");
        sliderMotor.setInverted(true);
        sliderMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setRunMode(Motor.RunMode.RawPower);

        // Register subsystems with CommandScheduler
        //CommandScheduler.getInstance().registerSubsystem(horizontalSlider);
        CommandScheduler.getInstance().registerSubsystem(verticalExtension);
        CommandScheduler.getInstance().registerSubsystem(outtakeArm);
        CommandScheduler.getInstance().registerSubsystem(gripper);
        CommandScheduler.getInstance().registerSubsystem(arm);

        outtakeArm.goToInit();
        arm.goToInit();

        outtakeArm.close();

        extendo.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }
}

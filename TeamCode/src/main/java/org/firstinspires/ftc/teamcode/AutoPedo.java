package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "AutoPedo")
public class AutoPedo extends OpMode {

    private DcMotor intake;
    private DcMotor spindex;
    private DcMotor flywheel;
    private Servo outtakeServo;
    private Servo heightServo;
    private DigitalChannel servoClosed;
    public double servoOut = 0;
    public double servoIn = 0.3;
    private int ticksBetween = 445;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(48, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(72, 24, Math.toRadians(28));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        spindex = hardwareMap.dcMotor.get("spindex");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeServo = hardwareMap.get(Servo.class, "outtake");
        heightServo = hardwareMap.get(Servo.class, "height");

        servoClosed = hardwareMap.get(DigitalChannel.class, "switch");
        servoClosed.setMode(DigitalChannel.Mode.INPUT);

        flywheel = hardwareMap.dcMotor.get("flywheel");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        flywheel.setPower(1);
        heightServo.setPosition(0.2);
        outtakeServo.setPosition(servoIn);
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }


    private Path scorePreload;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    outtakeServo.setPosition(servoOut);
                    wait(500);
                    outtakeServo.setPosition(servoIn);
                    wait(500);
                    spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spindex.setTargetPosition(-ticksBetween);
                    spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spindex.setPower(0.5);

                    while(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) > 5){}

                    outtakeServo.setPosition(servoOut);
                    wait(500);
                    outtakeServo.setPosition(servoIn);
                    wait(500);

                    spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spindex.setTargetPosition(-ticksBetween);
                    spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spindex.setPower(0.5);

                    while(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) > 5){}

                    outtakeServo.setPosition(servoOut);
                    wait(500);
                    outtakeServo.setPosition(servoIn);

                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


}

package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Autonomous(name = "AutoBlue")
public class AutoBlue extends OpMode {

    private DcMotor intake;
    private DcMotor spindex;
    private DcMotor flywheel;
    private Servo outtakeServo;
    private Servo heightServo;
    private DigitalChannel servoClosed;
    public double servoOut = 0;
    public double servoIn = 0.3;
    private int ticksBetween = 442;
    private double sleepTime = 400;
    private double flywheelPower = 0.95;
    int patternIndex = 21;
    private Limelight3A limelight3A;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(48, 9, Math.toRadians(0));
    private final Pose scorePose = new Pose(55, 15, Math.toRadians(20));
    private final Pose angle1Pose = new Pose(50, 33, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(37, 33, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(32, 33, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(9, 33, Math.toRadians(180));
    private final Pose _angle1Pose = new Pose(50, 57, Math.toRadians(180));
    private final Pose _pickup1Pose = new Pose(37, 57, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose _pickup2Pose = new Pose(32, 57, Math.toRadians(180));
    private final Pose _pickup3Pose = new Pose(9, 57, Math.toRadians(180));
    private final Pose finalPose = new Pose(40, 26, Math.toRadians(180));

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

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(5); //April Tags obelisk
        limelight3A.start();

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        heightServo.setPosition(0.2);
        outtakeServo.setPosition(servoIn);

        setPathState(0);
        flywheel.setPower(-flywheelPower);

    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }


    private Path scorePreload;
    private PathChain anglePickup1, grabPickup1, grabPickup2, grabPickup3, scorePath2, _anglePickup1, _grabPickup1, _grabPickup2, _grabPickup3, _scorePath2, finalPath;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        anglePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, angle1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), angle1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(angle1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(angle1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        _anglePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, _angle1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), _angle1Pose.getHeading())
                .build();

        _grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(_angle1Pose, _pickup1Pose))
                .setLinearHeadingInterpolation(_angle1Pose.getHeading(), _pickup1Pose.getHeading())
                .build();

        _grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(_pickup1Pose, _pickup2Pose))
                .setLinearHeadingInterpolation(_pickup1Pose.getHeading(), _pickup2Pose.getHeading())
                .build();

        _grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(_pickup2Pose, _pickup3Pose))
                .setLinearHeadingInterpolation(_pickup2Pose.getHeading(), _pickup3Pose.getHeading())
                .build();

        _scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(_pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(_pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        finalPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                LLResult result = limelight3A.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    patternIndex = fr.getFiducialId();
                }

                if(patternIndex == 22){
                    RunSpindexReverse();
                }else if(patternIndex == 23){
                    RunSpindex();
                }

                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    EmptySpindex();
                    intake.setPower(1);
                    flywheel.setPower(0);

                    follower.followPath(anglePickup1,true);
                    setPathState(2);
                }
                break;
            case 2:

                if(!follower.isBusy()){
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    Sleeping(800);
                    RunSpindex();
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    Sleeping(800);
                    RunSpindex();
                    follower.followPath(grabPickup3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    Sleeping(300);
                    intake.setPower(0);
                    flywheel.setPower(-flywheelPower);
                    follower.followPath(scorePath2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    EmptySpindex();
                    intake.setPower(1);
                    flywheel.setPower(0);

                    follower.followPath(_anglePickup1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(_grabPickup1);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    Sleeping(800);
                    RunSpindex();
                    follower.followPath(_grabPickup2);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    Sleeping(800);
                    RunSpindex();
                    follower.followPath(_grabPickup3);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    Sleeping(300);
                    intake.setPower(0);
                    flywheel.setPower(-flywheelPower);
                    follower.followPath(_scorePath2, true);
                    setPathState(11);
                }
            case 11:
                if(!follower.isBusy()){
                    EmptySpindex();
                    flywheel.setPower(0);

                    follower.followPath(finalPath, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void Sleeping(double ms){
        double time = getRuntime();
        while((getRuntime()-time) < (ms/1000)){
        }
    }
    public void RunSpindex(){
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(-ticksBetween);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(0.5);
    }
    public void RunSpindexReverse(){
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(ticksBetween);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(0.5);
    }

    public void EmptySpindex(){
        outtakeServo.setPosition(servoOut);
        Sleeping(sleepTime);
        outtakeServo.setPosition(servoIn);
        Sleeping(sleepTime-50);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(-ticksBetween);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(0.5);

        while(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) > 5){}

        outtakeServo.setPosition(servoOut);
        Sleeping(sleepTime);
        outtakeServo.setPosition(servoIn);
        Sleeping(sleepTime-50);

        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(-ticksBetween);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setPower(0.5);

        while(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) > 5){}

        outtakeServo.setPosition(servoOut);
        Sleeping(sleepTime);
        outtakeServo.setPosition(servoIn);

        Sleeping(300);
    }


}

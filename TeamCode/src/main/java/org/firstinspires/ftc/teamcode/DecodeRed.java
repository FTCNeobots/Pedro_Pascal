package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DecodeRed")
public class DecodeRed extends OpMode {

    private DcMotor intake;
    private DcMotor spindex;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor flywheel;
    private Servo outtakeServo;
    private Servo heightServo;
    private ColorSensor colorSensor;
    private double maxSpeed = 1;
    private double botHeading;
    private double turnSpeed = 1;
    private int position = 1;
    private int ticksBetween = 443;
    boolean spindexRunning = false;
    private int targetPosition = 1;
    private boolean ballAt1 = false;
    private boolean ballAt2 = false;
    private boolean ballAt3 = false;
    public boolean ballAtCurrentValue = false;
    public double servoOut = 0;
    public double servoIn = 0.3;
    private DigitalChannel servoClosed;
    public boolean timerOn = false;
    public boolean flywheelOn = true;
    private Limelight3A limelight3A;
    private double xCorrection = 0;
    private double yCorrection = 0;
    private boolean aimAssistInPosition = false;
    double time;
    double flywheelSpeed = -0.95;
    double flywheelSpeedFar = -0.95;
    double flywheelSpeedClose = -0.85;
    IMU imu;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        spindex = hardwareMap.dcMotor.get("spindex");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive = hardwareMap.dcMotor.get("leftBack");
        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightBack");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");

        outtakeServo = hardwareMap.get(Servo.class, "outtake");
        heightServo = hardwareMap.get(Servo.class, "height");

        servoClosed = hardwareMap.get(DigitalChannel.class, "switch");
        servoClosed.setMode(DigitalChannel.Mode.INPUT);

        flywheel = hardwareMap.dcMotor.get("flywheel");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");

        //rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //dit hoort niet te hoeven, maar zo werkt het?
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(4); //April Tags red

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-3.42, -6.77, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    }


    @Override
    public void start() {
        limelight3A.start();
        outtakeServo.setPosition(servoIn);
        heightServo.setPosition(0.2);

        colorSensor.enableLed(true);

        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pinpoint.update();


        if (gamepad1.back) {
            pinpoint.resetPosAndIMU();
            telemetry.addData("Yaw ", "reset!");

        }

        if(!((getRuntime() - time) < 1)){
            if(timerOn){
                outtakeServo.setPosition(servoIn);
                ballAtCurrentValue = false;
                if(position == 1){
                    ballAt2 = false;
                }
                if(position == 2){
                    ballAt3 = false;
                }
                if(position == 3){
                    ballAt1 = false;
                }
                timerOn = false;
            }
            if(!gamepad1.y){

                if(gamepad2.a){
                    spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    spindex.setPower(0.1);
                }else if(gamepad2.b){
                    spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    spindex.setPower(-0.1);
                }else{
                   SpindexCycling();
                   SpindexPositioning();
                }




            }else{
                spindex.setPower(0);
            }

        }


        if(gamepad1.a){
            AimAssist();
        }else{
            NormalDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            aimAssistInPosition = false;
        }



        HeightControl();
        ControlIntake();
        FlywheelControl();
        botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        telemetry.addData("AimAssist in position: ", aimAssistInPosition);
        telemetry.addData("Limit switch state: ", servoClosed.getState());

        telemetry.addData("Bot heading: ", pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Ball at 1: ", ballAt1);
        telemetry.addData("Ball at 2: ", ballAt2);
        telemetry.addData("Ball at 3: ", ballAt3);

        telemetry.update();
    }
    private void NormalDrive(double _Xget, double _Yget, double _Turnget) {

        if(gamepad1.left_trigger > 0){
            maxSpeed = -1;
            turnSpeed = 1;
        }else if(gamepad1.left_bumper){
            maxSpeed = -0.25;
            turnSpeed = 2;

        }else{
            maxSpeed = -0.5;
            turnSpeed = 2;
        }



        double _X = _Xget * Math.cos(botHeading) - _Yget * Math.sin(botHeading);
        double _Y = _Xget * Math.sin(botHeading) + _Yget * Math.cos(botHeading);
        ///die negatief hoort niet te hoeven, maar helpt wel
        double _Turn = -_Turnget * turnSpeed;
        _X = _X * 1.1;



        double _LFSpeed = MathLogic.Clamp(_Y - _X + _Turn, -1, 1) * maxSpeed;
        double _LBSpeed = MathLogic.Clamp(_Y + _X + _Turn, -1, 1) * maxSpeed;
        double _RBSpeed = MathLogic.Clamp(_Y - _X - _Turn, -1, 1) * maxSpeed;
        double _RFSpeed = MathLogic.Clamp(_Y + _X - _Turn, -1, 1) * maxSpeed;

        leftFrontDrive.setPower(_LFSpeed);
        leftBackDrive.setPower(_LBSpeed);
        rightBackDrive.setPower(_RBSpeed);
        rightFrontDrive.setPower(_RFSpeed);

    }
    private void ControlIntake(){
        if(gamepad1.right_trigger > 0){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }

    }

    private void SpindexPositioning(){
        if(!spindexRunning && servoClosed.getState()){
            int targetDifference = targetPosition - position;
            if(targetDifference == 1 || targetDifference == -2){
                spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindex.setTargetPosition(-ticksBetween);
                spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                position = targetPosition;
                spindexRunning = true;
            }
            if(targetDifference == 2 || targetDifference == -1){
                spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindex.setTargetPosition(ticksBetween);
                spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                position = targetPosition;
                spindexRunning = true;
            }
        }
        if(spindexRunning){
            spindex.setPower(0.5);
            if(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) < 5){
                spindexRunning = false;

            }
        }else{
            spindex.setPower(0);
            spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    private void SpindexCycling(){

        //moves the spindexer to a new intake position while saving that a ball is in the current position
        if((gamepad1.right_bumper || BallAccordingToColorSensor()) && !spindexRunning){
            //change state of current ball location
            if(position == 1){
                ballAt1 = true;
            }
            if(position == 2){
                ballAt2 = true;
            }
            if(position == 3){
                ballAt3 = true;
            }
            //move to empty ball location
            if(!ballAt3){
                targetPosition = 3;
            }else if(!ballAt2){
                targetPosition = 2;
            }else if (!ballAt1){
                targetPosition = 1;
            }else{
                telemetry.addData("Spindexer is ", "full");
            }

        }

        //moves the spindexer to a new intake position without saving that a ball is in the current position
        /*if(gamepad1.left_bumper && !spindexRunning){
            //move to empty ball location
            if(position == 1) {
                if (!ballAt2) {
                    targetPosition = 2;
                }
                if (ballAt3) {
                    targetPosition = 3;
                }
            }
            if(position == 2) {
                if (!ballAt1) {
                    targetPosition = 1;
                }
                if (ballAt3) {
                    targetPosition = 3;
                }
            }
            if(position == 3) {
                if (!ballAt2) {
                    targetPosition = 2;
                }
                if (ballAt1) {
                    targetPosition = 1;
                }
            }



        }*/

        //ejects a ball and saves that the current slot is empty
        /// to do: add the correct servo positions
        if(gamepad1.b || aimAssistInPosition){
            //checks if the current position is holding a ball
            if(!ballAtCurrentValue) {
                if (position == 1) {
                    if (ballAt2) {
                        ballAtCurrentValue = true;
                    }else{
                        //no ball at the current value, will move to a value with a ball
                        if(ballAt1){
                            targetPosition = 3;
                        }else if(ballAt2){
                            targetPosition = 1;
                        }else if (ballAt3){
                            targetPosition = 2;
                        }else{
                            telemetry.addData("No ", "balls!");
                        }
                    }
                }else if (position == 2) {
                    if (ballAt3) {
                        ballAtCurrentValue = true;
                    }else{
                        //no ball at the current value, will move to a value with a ball
                        if(ballAt1){
                            targetPosition = 3;
                        }else if(ballAt2){
                            targetPosition = 1;
                        }else if (ballAt3){
                            targetPosition = 2;
                        }else{
                            telemetry.addData("No ", "balls!");
                        }
                    }
                }else if (position == 3) {
                    if (ballAt1) {
                        ballAtCurrentValue = true;
                    }else{
                        //no ball at the current value, will move to a value with a ball
                        if(ballAt1){
                            targetPosition = 3;
                        }else if(ballAt2){
                            targetPosition = 1;
                        }else if (ballAt3){
                            targetPosition = 2;
                        }else{
                            telemetry.addData("No ", "balls!");
                        }
                    }
                }
            }
            //ejects the ball if there is one
            if(ballAtCurrentValue && !spindexRunning){
                //servo position push
                outtakeServo.setPosition(servoOut);
                time = getRuntime();
                timerOn = true;

            }
        }

    }
    public void FlywheelControl(){

        if(gamepad1.dpad_up){
            flywheelOn = true;
        }else if(gamepad1.dpad_down){
            flywheelOn = false;
        }
        if(flywheelOn){
            flywheel.setPower(flywheelSpeed);
        }else{
            flywheel.setPower(0);
        }

        if(gamepad2.x){
            flywheelSpeedFar = -0.95;
            flywheelSpeedClose = -0.85;

            if(flywheel.getPower() == -0.95 || flywheel.getPower() == -1){
                flywheelSpeed = flywheelSpeedFar;
            }
            if(flywheel.getPower() == -0.9 || flywheel.getPower() == -0.85){
                flywheelSpeed = flywheelSpeedClose;
            }

        }
        if(gamepad2.y){
            flywheelSpeedFar = -1;
            flywheelSpeedClose = -0.9;

            if(flywheel.getPower() == -0.95 || flywheel.getPower() == -1){
                flywheelSpeed = flywheelSpeedFar;
            }
            if(flywheel.getPower() == -0.9 || flywheel.getPower() == -0.85){
                flywheelSpeed = flywheelSpeedClose;
            }
        }

        if(gamepad1.dpad_left){
            flywheelSpeed = flywheelSpeedFar;
        }else if(gamepad1.dpad_right){
            flywheelSpeed = flywheelSpeedClose;
        }


    }
    public void HeightControl(){
        heightServo.setPosition(0.2);
    }


    public void AimAssist(){
        double pX = 0.015;
        double targetYaw = -45 * 3.141592654 / 180;
        double targetX;
        double targetA;
        double feedforward = 0.05;
        double deadZone = 2;
        double positionFar = 0.5;
        double positionClose = 0.7;
        double offsetX = -5 ;
        maxSpeed = -1;


        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null && llResult.isValid()){
            if(llResult.getTa() < 0.5){
                targetX = 0;
                targetA = 0.34;
                heightServo.setPosition(0.2);
                flywheelSpeed = flywheelSpeedFar;

            }else{
                targetX = 0;
                targetA = 0;
                heightServo.setPosition(0.2);
                flywheelSpeed = flywheelSpeedClose;
            }

            if((llResult.getTx() - targetX) > (deadZone + offsetX)){

                xCorrection = feedforward + (llResult.getTx() - targetX) * pX;
                aimAssistInPosition = false;

            }else if((llResult.getTx() - targetX) < (-deadZone + offsetX)){

                xCorrection = -feedforward + (llResult.getTx() - targetX) * pX;
                aimAssistInPosition = false;

            }else{
                xCorrection = 0;
                yCorrection = 0;


                aimAssistInPosition = true;
            }
        }else{
            if((targetYaw) < botHeading && (3.141592654 + targetYaw) > botHeading){
                xCorrection = 0.5;

            }else{
                xCorrection = -0.5;

            }

            yCorrection = 0;

            aimAssistInPosition = false;
        }



        double _LFSpeed = MathLogic.Clamp(yCorrection - xCorrection, -1, 1) * maxSpeed;
        double _LBSpeed = MathLogic.Clamp(yCorrection - xCorrection, -1, 1) * maxSpeed;
        double _RBSpeed = MathLogic.Clamp(yCorrection + xCorrection, -1, 1) * maxSpeed;
        double _RFSpeed = MathLogic.Clamp(yCorrection + xCorrection, -1, 1) * maxSpeed;

        leftFrontDrive.setPower(_LFSpeed);
        leftBackDrive.setPower(_LBSpeed);
        rightBackDrive.setPower(_RBSpeed);
        rightFrontDrive.setPower(_RFSpeed);

    }

    public boolean BallAccordingToColorSensor(){

        if(colorSensor.alpha() >= 3){
            return true;
        }else{
            return false;
        }

    }




}

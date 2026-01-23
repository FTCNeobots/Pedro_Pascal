package org.firstinspires.ftc.teamcode;

import androidx.core.view.WindowInsetsAnimationCompat;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "TestMotors")
public class TestMotors extends OpMode {
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override
    public void init() {
        leftBackDrive = hardwareMap.dcMotor.get("leftBack");
        leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        rightBackDrive = hardwareMap.dcMotor.get("rightBack");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            leftBackDrive.setPower(1);
        }else{
            leftBackDrive.setPower(0);
        }
        if(gamepad1.b){
            leftFrontDrive.setPower(1);
        }else{
            leftFrontDrive.setPower(0);
        }
        if(gamepad1.x){
            rightBackDrive.setPower(1);
        }else{
            rightBackDrive.setPower(0);
        }
        if(gamepad1.y){
            rightFrontDrive.setPower(1);
        }else{
            rightFrontDrive.setPower(0);
        }


    }
}

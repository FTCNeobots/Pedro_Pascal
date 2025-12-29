package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TestSingleMotor")
public class TestSingleMotor extends LinearOpMode {

    private DcMotor intake;
    private DcMotor spindex;
    private double speedI = 0.5;
    private double speedS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        spindex = hardwareMap.dcMotor.get("spindex");

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(speedI);
            }
            if (gamepad1.b) {
                intake.setPower(-speedI);
            }
            if (gamepad1.x) {
                spindex.setPower(speedS);
            }
            if (gamepad1.y) {
                spindex.setPower(-speedS);
            }
            if(gamepad1.left_bumper){
                speedI = 0.5;
            }
            if(gamepad1.right_bumper){
                speedI = 1;
            }
            if(gamepad1.left_trigger > 0){
                speedS = 0.5;
            }
            if(gamepad1.right_trigger > 0){
                speedS = 1;
            }

            StopMoving();
        }

    }
    public void StopMoving(){
        intake.setPower(0);
        spindex.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "test")
public class TestSingleMotor extends LinearOpMode {

    private DcMotor testMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.dcMotor.get("test");

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                testMotor.setPower(0.5);
            }
            if (gamepad1.b) {
                testMotor.setPower(-0.5);
            }
            if (gamepad1.x) {
                testMotor.setPower(1);
            }
            if (gamepad1.y) {
                testMotor.setPower(-1);
            }
            testMotor.setPower(0);
        }

    }
}

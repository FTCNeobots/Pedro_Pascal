package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SpindexPositioning")
public class SpindexPositioning extends OpMode {
    private int position = 1;
    private int ticksBetween = 445;
    boolean spindexRunning = false;

    private DcMotor spindex;
    private int targetPosition = 1;
    @Override
    public void init() {
        spindex = hardwareMap.dcMotor.get("spindex");
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            targetPosition = 1;
        }
        if(gamepad1.b){
            targetPosition = 2;
        }
        if(gamepad1.x){
            targetPosition = 3;
        }
        SpindexPositioning();

        telemetry.addData("position: ", position);
        telemetry.update();
    }

    private void SpindexPositioning(){
        if(!spindexRunning){
            spindex.setPower(0);
            int targetDifference = targetPosition - position;
            if(targetDifference == 1 | targetDifference == -2){
                spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindex.setTargetPosition(ticksBetween);
                spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                position = targetPosition;
                spindexRunning = true;
            }
            if(targetDifference == 2 | targetDifference == -1){
                spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spindex.setTargetPosition(-ticksBetween);
                spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                position = targetPosition;
                spindexRunning = true;
            }

        }else{
            spindex.setPower(1);
            if(abs(spindex.getCurrentPosition() - spindex.getTargetPosition()) < 5){
                spindexRunning = false;
            }

        }


    }


}

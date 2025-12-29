package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimeLightTest")
public class LimeLightTest extends OpMode {

    private Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(1); //green


    }
    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null && llResult.isValid()){
            telemetry.addData("X offset: ", llResult.getTx());
            telemetry.addData("Y offset: ", llResult.getTy());
            telemetry.addData("Area offset: ", llResult.getTa());
        }



        telemetry.update();
    }
}

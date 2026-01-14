package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LimelightAngleTune")
public class LimelightAngleTune extends OpMode {

    private Limelight3A limelight3A;


    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3); //April Tags blue
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {



        LLResult llResult = limelight3A.getLatestResult();
        //if(llResult != null && llResult.isValid()){
            telemetry.addData("tX = ", llResult.getTx());
            telemetry.addData("tY = ", llResult.getTy());
            telemetry.addData("tA = ", llResult.getTa());
        telemetry.update();


    }
}

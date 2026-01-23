package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "LimelightAngleTune")
public class LimelightAngleTune extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor flywheel;


    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        limelight3A.pipelineSwitch(3); //April Tags blue
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {

        flywheel.setPower(-1);

        LLResult llResult = limelight3A.getLatestResult();
        //if(llResult != null && llResult.isValid()){
            telemetry.addData("tX = ", llResult.getTx());
            telemetry.addData("tY = ", llResult.getTy());
            telemetry.addData("tA = ", llResult.getTa());
        telemetry.update();


    }
}

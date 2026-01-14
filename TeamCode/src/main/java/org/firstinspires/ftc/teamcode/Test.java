package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

@TeleOp(name = "Test")
public class Test extends OpMode {

    public Servo servo;
    //public ColorSensor sensor;
    public DcMotor flywheel;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("outtake");
        //sensor = hardwareMap.get(ColorSensor.class, "sensor");

        flywheel = hardwareMap.dcMotor.get("flywheel");



    }

    @Override
    public void loop() {

        //sensor.enableLed(true); //is toch mooi?

        //telemetry.addData("Color number: ", sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        //telemetry.addData("Color index: ", sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX));
        //telemetry.addData("Red: ", sensor.red());
        //telemetry.addData("blue: ", sensor.blue());
        //telemetry.addData("green: ", sensor.green());



        telemetry.update();

        if(gamepad1.left_bumper){
            flywheel.setPower(-0.9);
        }
        if(gamepad1.right_bumper){
            flywheel.setPower(-1);
        }
        if(gamepad1.left_trigger > 0){
            flywheel.setPower(-0.7);
        }
        if(gamepad1.right_trigger > 0){
            flywheel.setPower(-0.8);
        }


        if(gamepad1.a){
            servo.setPosition(0.2);
        }
        if (gamepad1.b){
            servo.setPosition(0.8);
        }
        if(gamepad1.x){
            servo.setPosition(0.4);
        }
        if (gamepad1.y){
            servo.setPosition(0.6);
        }
    }
}

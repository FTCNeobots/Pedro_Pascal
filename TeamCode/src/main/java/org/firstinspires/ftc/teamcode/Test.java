package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;



@TeleOp(name = "Test")
public class Test extends OpMode {

    public ColorSensor sensor;

    @Override
    public void init() {

        sensor = hardwareMap.get(ColorSensor.class, "sensor");




    }

    @Override
    public void loop() {

        sensor.enableLed(true); //is toch mooi?

        telemetry.addData("Red: ", sensor.red());
        telemetry.addData("Green: ", sensor.green());
        telemetry.addData("Blue: ", sensor.blue());
        telemetry.addData("ARGB: ", sensor.argb());
        telemetry.addData("Alpha: ", sensor.alpha());



        telemetry.update();


    }
}

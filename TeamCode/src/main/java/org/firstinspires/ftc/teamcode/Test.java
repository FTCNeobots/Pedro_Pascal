package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

@TeleOp(name = "Test")
public class Test extends OpMode {

    public Servo servo;
    public ColorSensor sensor;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");


    }

    @Override
    public void loop() {

        sensor.enableLed(true); //is toch mooi?

        //telemetry.addData("Color number: ", sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        //telemetry.addData("Color index: ", sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_INDEX));
        telemetry.addData("Red: ", sensor.red());
        telemetry.addData("blue: ", sensor.blue());
        telemetry.addData("green: ", sensor.green());
        telemetry.update();

        if(gamepad1.a){
            servo.setPosition(0.2);
        }
        if (gamepad1.b){
            servo.setPosition(0.8);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoExample extends OpMode {

    private Servo servo0;

    @Override
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo0.setPosition(0);
        } else if (gamepad1.b) {
            servo0.setPosition(1);
        }
    }

}

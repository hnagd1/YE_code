package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class helloWorld extends OpMode {

    private int value = 1;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Value", value++);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}
//World

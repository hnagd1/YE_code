package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class encoderReset extends OpMode {
    DcMotor Motor4;
    DcMotor Motor5;
    DcMotor Motor6;
    @Override
    public void init() {
        Motor4 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor6 = hardwareMap.get(DcMotor.class, "Motor6");

        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motor4.setTargetPosition(0);
        Motor5.setTargetPosition(0);
        Motor6.setTargetPosition(0);

        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {

    }
}

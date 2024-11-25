package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class robotTeleOpTriggerTest extends OpMode {

    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;

    private double lx;
    private double ly;
    private double rx;

    private double turboTrigger;


    @Override
    public void init() {
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        lx = gamepad1.left_stick_x * 1.1; //Strafe variable
        rx = -gamepad1.left_stick_y; //Forward and backward
        ly = -gamepad1.right_stick_x; //Turn right and left
        turboTrigger = 1 - gamepad1.right_trigger;

        Motor0.setPower(turboTrigger * (ly + rx + lx));
        Motor1.setPower(turboTrigger * (-ly + (rx - lx)));
        Motor2.setPower(turboTrigger * (-ly + rx + lx));
        Motor3.setPower(turboTrigger * (ly + (rx - lx)));

    }
}

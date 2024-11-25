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

    private double lx; //creates variable lx to store the value of the input on the left stick's horizontal axis
    private double ly; //creates variable ly to store the value of the input on the left stick's vertical axis
    private double rx; //creates variable rx to store the value of the input on the right stick's horizontal axis

    private double turboTrigger;


    @Override
    public void init() {
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    } // gets the hardware map sets up motor direction for the mecanum wheels

    @Override
    public void loop() {
        lx = gamepad1.left_stick_x * 1.1; //Multiplies the input from the horizontal left stick on gamepad 1 by the strafe variable to account for wheel-y-ness
        rx = -gamepad1.left_stick_y; //Uses the left stick's vertical input to move forward and backward
        ly = -gamepad1.right_stick_x; //Uses the right stick's horizontal input to turn right and left
        turboTrigger = 1 - gamepad1.right_trigger; //Sets turboTrigger to the opposite of the input from the trigger so that all the way down is no power instead of full power

        Motor0.setPower(turboTrigger * (ly + rx + lx));
        Motor1.setPower(turboTrigger * (-ly + (rx - lx)));
        Motor2.setPower(turboTrigger * (-ly + rx + lx));
        Motor3.setPower(turboTrigger * (ly + (rx - lx)));
        //Sets powers of the motors normally for mecanum wheels but multiplies it by turboTrigger so the input from the trigger effects(affects?) the speed of the robot
    }
}

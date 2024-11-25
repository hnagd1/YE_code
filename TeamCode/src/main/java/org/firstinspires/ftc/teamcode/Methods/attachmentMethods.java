package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class attachmentMethods {

    DcMotor Motor4;
    CRServo Servo0;
    Servo Servo1;

    // Initilize any hardware maps, set dirictions, and set up encoders
    // TODO: test the setPower extention code
    public void init(HardwareMap hardwareMap) {
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        Servo0 = hardwareMap.get(CRServo.class, "Servo0");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");

        Motor4.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo0.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servo1.setPosition(0.8);

        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toggleLiftArm(double ry, Telemetry telemetry) {
        if (ry > 0) {
            Motor4.setDirection(DcMotorSimple.Direction.FORWARD);
            Motor4.setPower(Math.abs(ry));
            telemetry.addData("Movment","up");
        } else if (ry < 0) {
            Motor4.setDirection(DcMotorSimple.Direction.REVERSE);
            if (Motor4.getCurrentPosition()<-1376) {
                Motor4.setPower(Math.abs(ry));
            } else {
                Motor4.setPower(0);
            }
            telemetry.addData("Movment","down");
        } else if (ry < 0.1 && ry > -0.1) {
            Motor4.setPower(0);
            telemetry.addData("Movment","none");

        }
        telemetry.addData("Position",Motor4.getCurrentPosition());
//1548
    }

    public void rotateLiftArm(int direction, int position, Telemetry telemetry) {

        if (direction == 1) {Motor4.setDirection(DcMotorSimple.Direction.FORWARD);} else {Motor4.setDirection(DcMotorSimple.Direction.REVERSE);}
        Motor4.setTargetPosition(position);
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Motor4.isBusy()) {Motor4.setPower(1);}
        Motor4.setPower(0);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void toggleIntake(double power) {
        if (power == 0) {
            Servo0.setPower(0);
        } else if (power > 0) {
            Servo0.setDirection(DcMotorSimple.Direction.FORWARD);
            Servo0.setPower(power);
        } else {
            Servo0.setDirection(DcMotorSimple.Direction.REVERSE);
            Servo0.setPower(Math.abs(power));
        }
    }
    // Toggle on/off intake


    public void resetServo() {
        Servo1.setPosition(0);
    }

}

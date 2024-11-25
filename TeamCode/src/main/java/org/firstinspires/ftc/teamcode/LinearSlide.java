package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {

    private DcMotor Motor5;
    private DcMotor Motor6;

    private Servo Servo1;

    public void init(HardwareMap hardwareMap) {
        Servo1 = hardwareMap.get(Servo.class, "Servo1");

        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor5.setDirection(DcMotorSimple.Direction.REVERSE);

        Motor6 = hardwareMap.get(DcMotor.class, "Motor6");
        Motor6.setDirection(DcMotorSimple.Direction.REVERSE);

        Motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop(boolean LB, boolean RB, boolean active, Telemetry telemetry) {
        if (active) {
            if (LB) {

                Motor5.setDirection(DcMotorSimple.Direction.REVERSE);
                Motor6.setDirection(DcMotorSimple.Direction.FORWARD);
                Motor5.setTargetPosition(3100);
                Motor6.setTargetPosition(3100);
                if (Motor5.getCurrentPosition() <= 3100) {
                    Motor5.setPower(0.7);
                    Motor6.setPower(0.7);
                } else if (Motor5.getCurrentPosition() > 3100) {
                    Motor5.setPower(0);
                    Motor6.setPower(0);
                }

                Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (RB) {

                Motor5.setDirection(DcMotorSimple.Direction.FORWARD);
                Motor6.setDirection(DcMotorSimple.Direction.REVERSE);
                Motor5.setTargetPosition(0);
                Motor6.setTargetPosition(0);
                telemetry.addData("Value", Math.abs(Motor5.getCurrentPosition()));
                if (Math.abs(Motor5.getCurrentPosition()) >= 0) {
                    telemetry.addData("This function has been called", 1);
                    Motor5.setPower(0.7);
                    Motor6.setPower(0.7);
                } else if (Math.abs(Motor5.getCurrentPosition()) < 0) {
                    Motor5.setPower(0);
                    Motor6.setPower(0);

                }

                Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!LB && !RB) {
                Motor5.setPower(0);
                Motor6.setPower(0);
            }
        }
    }
    //TODO take code from loop function and put it in extendLinearSlide and contractLinearSlide, replace code in loop with function calls
    public void extendLinearSlide(Telemetry telemetry) {
        Motor5.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor6.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor5.setTargetPosition(3100);
        Motor6.setTargetPosition(3100);

        if (Motor5.getCurrentPosition() <= 3100) {
            Motor5.setPower(0.7);
            Motor6.setPower(0.7);
        } else if (Motor5.getCurrentPosition() > 3100) {
            Motor5.setPower(0);
            Motor6.setPower(0);
        }
        Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void contractLinearSlide(Telemetry telemetry) {
        Motor5.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor6.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor5.setTargetPosition(3100);
        Motor6.setTargetPosition(3100);
        telemetry.addData("Value", Math.abs(Motor5.getCurrentPosition()));
        if (Math.abs(Motor5.getCurrentPosition()) <= 3100) {
            telemetry.addData("This function has been called", 1);
            Motor5.setPower(0.7);
            Motor6.setPower(0.7);
        } else if (Math.abs(Motor5.getCurrentPosition()) > 3100) {
            Motor5.setPower(0);
            Motor6.setPower(0);

        }

        Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void basketPos(double pos) {
        Servo1.setPosition(pos);
    }

    public void basketServo(boolean x) {
        if (x) {
            Servo1.setPosition(1);
        } else {
            Servo1.setPosition(0);
        }
    }

    public void stop() {
        Motor5.setPower(0);
        Motor6.setPower(0);
    }
}
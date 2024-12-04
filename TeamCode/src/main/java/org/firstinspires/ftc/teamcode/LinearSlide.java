package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    //Define Motors and Servos
    private DcMotor Motor5;
    private DcMotor Motor6;

    private Servo Servo1;

    public void init(HardwareMap hardwareMap) {
        //Apply hardware maps
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor6 = hardwareMap.get(DcMotor.class, "Motor6");
        //This reverses the motors so that Motors 6's encoder values are reversed.
        //This helps because if both motor 6 has to rotate in the opposite direction as motor 5
        //So it has to have the opposite encoder values to be able to run to position.
        Motor5.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor6.setDirection(DcMotorSimple.Direction.REVERSE);
        //Reset the encoders on the motors so that they are 0 at this position (init position)
        Motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //This sets the target position to 0
        //This is to prevent a crash when there is no target position and the motors are set to run to position
        Motor5.setTargetPosition(0);
        Motor6.setTargetPosition(0);
        Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop(boolean LB, boolean RB, boolean active, Telemetry telemetry) {
        if (active) {
            if (LB) {
                runLinearSlide(0.7,telemetry);
            }
            if (RB) {
                runLinearSlide(-0.7,telemetry);
            }
            if (!LB && !RB) {
                runLinearSlide(0.0,telemetry);
            }
        }
    }

    public void runLinearSlide(Double power, Telemetry telemetry) {
        if (power < 0) {
            Motor5.setTargetPosition(0);
            Motor6.setTargetPosition(0);
        } else {
            Motor5.setTargetPosition(3100);
            Motor6.setTargetPosition(3100);
        }
        Motor5.setPower(Math.abs(power));
        Motor6.setPower(Math.abs(power));
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
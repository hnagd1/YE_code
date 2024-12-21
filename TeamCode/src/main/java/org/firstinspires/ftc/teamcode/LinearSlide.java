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

    Servo Servo2;

    public void init(HardwareMap hardwareMap) {
        //Apply hardware maps
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
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

    public void loop(boolean LB, boolean RB, boolean active, Telemetry telemetry, boolean left, boolean right) {
        /* This is the main loop for linear slides. This loop contains 4 operands:
        bool LB - Left bumper on gamepad 2. Raises the linear slides
        bool RB - Right bumper on gamepad 2 Lowers the linear slides
        bool active - used to disable the linear slides when they are automatically being lowered
        telemetry - this is so we can use telemetry in this function
         */
        telemetry.addData("LS",Motor5.getCurrentPosition());
        if (left) {
            Servo2.setPosition(Servo2.getPosition()-0.0025);
        } else if (right) {
            Servo2.setPosition(Servo2.getPosition()+0.0025);
        } else {
            Servo2.setPosition(Servo2.getPosition());
        }
        telemetry.addData("SERVO2",Servo2.getPosition());

        if (active) { //Checks if this loop is active
            if (LB) {
                //Set the linear slide power to 0.7 if LB is pressed to raise
                runLinearSlide(0.7,telemetry);
            } else if (RB) {
                //Set the linear slide power to -0.7 if RB is pressed to lower
                runLinearSlide(-0.7,telemetry);
            } else {
                //stop the linear slides if nothing is being pressed
                runLinearSlide(0.0,telemetry);
            }
        }
    }

    public void runLinearSlide(Double power, Telemetry telemetry) {
        if (power < 0) { //sets the target position based on if it is negative or positive
            Motor5.setTargetPosition(0);
            Motor6.setTargetPosition(0);
        } else if (power > 0) {
            Motor5.setTargetPosition(3100);
            Motor6.setTargetPosition(3100);
        }
        //sets it to the abs of power so that the power is not negative when going down
        Motor5.setPower(Math.abs(power));
        Motor6.setPower(Math.abs(power));
    }

    public void basketPos(double pos) { //The purpose of this function is so that the main teleop can set the position of the servo
        Servo2.setPosition(pos);
    }

    public void stop() { //sets the linear slide powers to 0
        Motor5.setPower(0);
        Motor6.setPower(0);
    }
}

//
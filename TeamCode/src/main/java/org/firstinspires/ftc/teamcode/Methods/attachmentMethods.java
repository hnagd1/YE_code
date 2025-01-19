package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class attachmentMethods {

    // Creates a variable for Motor 4 (Arm motor), continuous rotating severo 0 (Intake servo), and Servo 1 (use changes, mostly used as an arm joint)
    DcMotor Motor4;
    CRServo Servo0;
    Servo Servo1;
    // Servo2 can be found in linear slide code
    Servo Servo3;
    Servo Servo4;

    // TODO: test the setPower extention code
    public void init(HardwareMap hardwareMap) {

        // Uses hardwareMap.get to intilize the Motor and servos lifted above
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        Servo0 = hardwareMap.get(CRServo.class, "Servo0");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        Servo4 = hardwareMap.get(Servo.class, "Servo4");

        // Sets the starting diriction of the arm motor and intake servo, this will change as the drivers control the robot
        Motor4.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo0.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sets the zero power behavior to BRAKE in an effort to stop the arm from moving when inputs are not giving to it. - Effectiveness is unknown
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Resets the encoder to position 0 and sets the mode so that the encoder isn't interfering with Motor inputs
        // Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Function is used for driver controled arm movments
    public void toggleLiftArm(double triggers, Telemetry telemetry) {
        telemetry.addData("Pos:", Motor4.getCurrentPosition());
        /* This function takes the parameters:
                1. Triggers, the value of gamepad 2 left trigger - gamepad 2 right trigger
                2. Telemetry, a class used to write to the console. Required to be passed in as this function isn't part of a OpMode
         */

        // Checks if the triggers value is more than 0 (left trigger is being pressed down more than right trigger)
        if (triggers > 0) {
            // Sets the direction of the motor to forward, or moving away from the robot
            Motor4.setDirection(DcMotorSimple.Direction.REVERSE);
            Motor4.setPower(Math.abs(triggers));
        // Checks if the triggers value is less than 0 (right trigger is being pressed down more than left trigger)
        } else if (triggers < 0) {
            // Sets the direction of the motor to REVERSE, or moving to the robot
            Motor4.setDirection(DcMotorSimple.Direction.FORWARD);
            Motor4.setPower(Math.abs(triggers));
        // Checks if triggers are not pressed down or pressed down very little
        } else if (triggers < 0.1 && triggers > -0.1) {
            // Sets power to 0 because no buttons are being pressed and the motor should thus not be moving
            Motor4.setPower(0);

        }
    }

    // Function is used for automatic arm movement without any driver interaction
    public void rotateLiftArm(int direction, int position, Telemetry telemetry) {
        // The desired direction (as an int), position (as an int), and telemetry class are passed into this function
        // TODO: switch the direction to a boolean, as we are only going to be passing in 1 or 0

        // Checks if a 1 is passed in via the direction parameter
        if (direction == 1) {
            // Sets the arm direction to FORWARD, which is moving away from the robot
            Motor4.setDirection(DcMotorSimple.Direction.FORWARD);
        // Checks if a 0 is passed in via the direction parameter (really any number other than 1 would work, but for best practices just use 0)
        } else {
            // Sets the arm direction to REVERSE, which is moving toward the robot
            Motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Uses the passed in position to set the encoder target position
        Motor4.setTargetPosition(position);
        // Sets the Arm motor to run to position mode, which will move the arm until it reaches the target position
        Motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Applies power to the Motor and waits until it is done moving
        while(Motor4.isBusy()) {Motor4.setPower(1);}
        // After motor reaches destination it sets the power to 0 and comes to a stop
        Motor4.setPower(0);
        // Resets the encoder value to 0
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Used to control the direction of the intake motor
    public void toggleIntake(double power) {
        // Takes a power parameter which will control the intake wheels power

        // Checks if power that is passed in to 0 or none
        if (power == 0) {
            // Sets power of the servo to 0
            Servo0.setPower(0);
        // Checks if the passed in power is greater than 0
        } else if (power > 0) {
            // Sets direction to forward and runs using the passed in power value
            Servo0.setDirection(DcMotorSimple.Direction.FORWARD);
            Servo0.setPower(power);
        // If the power value is negative this is called
        } else {
            // Sets direction to REVERSE and runs using the passed in power value
            Servo0.setDirection(DcMotorSimple.Direction.REVERSE);
            Servo0.setPower(Math.abs(power));
        }
    }

    public void jointMovement(boolean dpd, boolean dpu, Telemetry telemetry) {
        telemetry.addData("INTAKE MOTOR",Motor4.getCurrentPosition());
        telemetry.addData("INTAKE SERVO",Servo1.getPosition());
        // Function passes in dpd (boolean of if d-pad down is preased) and dpu (boolean of if d-pad up is preased) and the telemetry class
        if (dpd) {
            Servo1.setPosition(Servo1.getPosition()-0.0025);
        } else if (dpu) {
            Servo1.setPosition(Servo1.getPosition()+0.0025);
        } else {
            Servo1.setPosition(Servo1.getPosition());
        }
        telemetry.addData("Servo", Servo4.getPosition());
    }

    public void setServoPosition(int servoNum, double position) {
        if (servoNum == 1) {
            Servo1.setPosition(position);
        } else if (servoNum == 3) {
            Servo3.setPosition(position);
        } else if (servoNum == 4) {
            Servo4.setPosition(position);
        }
    }

    public void teleLoop(Telemetry telemetry) {
        telemetry.addData("Servo1", Servo1.getPosition());
    }

    // Used for servo1 init stuff
    // TODO: confirm where this is still being used
    public void resetServo() {
        Servo1.setPosition(0);
    }

}

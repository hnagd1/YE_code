package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;


@Autonomous
public class basicAutonomous extends OpMode {
    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    LinearSlide linearSlide = new LinearSlide();
    attachmentMethods arm = new attachmentMethods(); //Makes instances of all the methods classes that we made so we can use the functions in them

    private DcMotor Motor5;
    private DcMotor Motor6;

    private Servo Servo1; //Sets up variables for the new motors and servos that aren't in drivingMethods so we can unfold

    @Override
    public void init() {
        auto.init(hardwareMap); //runs init in autonomousMethods to set up the motors

        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor6 = hardwareMap.get(DcMotor.class, "Motor6"); //sets up hardwareMap for the linear slide motors so you can extend the linear slides

        Servo1 = hardwareMap.get(Servo.class, "Servo1"); //sets up hardwareMap for the bucket servo so it can rotate to put samples in the buckets

        Motor5.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor6.setDirection(DcMotorSimple.Direction.REVERSE); //Makes the motors run in the same direction due to how they're set up on the robot
    }

    @Override
    public void start() {
        resetRuntime(); //sets RunTime to 0

        while (getRuntime() < 2.5) {
            auto.setDriveAngle(90, 1); //drives into wall (maybe we should make this NOT full speed?) until it has been running for 2.5 seconds to park in observation zone
        }
        auto.stopMotors();
        telemetry.addData("Stage:","done");
    }

    public void unfold() {
        try {
            Motor5.setPower(0.5);
            Motor6.setPower(0.5);
            Thread.sleep(1000); //fully extends linear slides
            Motor5.setPower(0);
            Motor6.setPower(0);
            Servo1.setPosition(0);
            Thread.sleep(500); //stops linear slides and sets the basket's current position to position 0
            Servo1.setPosition(0.85);
            arm.toggleLiftArm(1, telemetry);
            Thread.sleep(1000); //moves basket and intake to correct position
            arm.toggleLiftArm(0, telemetry); //stops arm movement
            Motor5.setDirection(DcMotorSimple.Direction.REVERSE);
            Motor6.setDirection(DcMotorSimple.Direction.FORWARD); //reverses the direction of the motors so you can move the linear slide back down
            Motor5.setPower(0.5);
            Motor6.setPower(0.5);
            Thread.sleep(1000); //moves linear slide back down
            Motor5.setPower(0);
            Motor6.setPower(0); //stops linear slide
            }
        catch(InterruptedException e) {
            telemetry.addLine("unfold had an exception :(");
        } //try catch to stop errors from the sleeps
    }

    public void fold() {
        //extend linear slide
        //Move arm back
        //un-extend linear slide
    } //folds the robot back to the starting position for easy setup in between matches

    @Override
    public void loop() {
    } //loop that does nothing! Yay!

    @Override
    public void stop() {
        auto.stopMotors();

    } //makes the motors stop when you push stop.

    /* public void setDriveAngleFor(int angle, double speed, double time) {
        resetRuntime();

        while (getRuntime() < time) {
            auto.setDriveAngle(angle, speed);
        }
        auto.stopMotors();
        telemetry.addData("Stage:","done");
    } */
} //no idea hihihihihih

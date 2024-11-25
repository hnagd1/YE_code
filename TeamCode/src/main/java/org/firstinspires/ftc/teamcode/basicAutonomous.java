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
    attachmentMethods arm = new attachmentMethods();

    private DcMotor Motor5;
    private DcMotor Motor6;

    private Servo Servo1;

    @Override
    public void init() {
        auto.init(hardwareMap);

        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        Motor6 = hardwareMap.get(DcMotor.class, "Motor6");

        Servo1 = hardwareMap.get(Servo.class, "Servo1");

        Motor5.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor6.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        resetRuntime();

        while (getRuntime() < 2.5) {
            auto.setDriveAngle(90, 1);
        }
        auto.stopMotors();
        telemetry.addData("Stage:","done");
    }

    public void unfold() {
        try {
            Motor5.setPower(0.5);
            Motor6.setPower(0.5);
            Thread.sleep(1000);
            Motor5.setPower(0);
            Motor6.setPower(0);
            Servo1.setPosition(0);
            Thread.sleep(500);
            Servo1.setPosition(0.85);
            arm.toggleLiftArm(1, telemetry);
            Thread.sleep(1000);
            arm.toggleLiftArm(0, telemetry);
            Motor5.setDirection(DcMotorSimple.Direction.REVERSE);
            Motor6.setDirection(DcMotorSimple.Direction.FORWARD);
            Motor5.setPower(0.5);
            Motor6.setPower(0.5);
            Thread.sleep(1000);
            Motor5.setPower(0);
            Motor6.setPower(0);
            }
        catch(InterruptedException e) {
            telemetry.addLine("unfold had an exception :(");
        }
    }

    public void fold() {
        //extend linear slide
        //Move arm back
        //un-extend linear slide
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        auto.stopMotors();

    }

    /* public void setDriveAngleFor(int angle, double speed, double time) {
        resetRuntime();

        while (getRuntime() < time) {
            auto.setDriveAngle(angle, speed);
        }
        auto.stopMotors();
        telemetry.addData("Stage:","done");
    } */
}

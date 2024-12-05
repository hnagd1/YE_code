package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;


@TeleOp
public class robotTeleOpDrive extends OpMode {

    double yPressTime;
    boolean yPress;
    double aPressTime;
    boolean aPress;
    double slideContractTime;
    boolean slideContract;
    boolean brakeButton = false;
    boolean rSB = false;

    boolean IMUReset;


    //TODO: add comments on this
    drivingMethods drive = new drivingMethods();

    @Override
    public void init() {
        //initialize functions with the hardware map
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        fieldCentricDrive();
    }

    public void robotCentricDrive() {
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        drive.drive(lx, ly, rx);
    }

    public void fieldCentricDrive() {
        //TODO: test this
        IMUReset = gamepad1.options;
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        drive.fieldCentric(lx, ly, rx, IMUReset, telemetry);
    }

    public void stop() { //this needs to stop everything
        drive.setMode(true); //this sets the motors to brake
        drive.setPower(0,0,0,0); //Set motor powers to 0
    }
}



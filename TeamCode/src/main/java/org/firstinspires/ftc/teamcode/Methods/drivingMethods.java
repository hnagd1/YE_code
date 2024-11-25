package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivingMethods {

    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;

    private double motorSpeed;
    double power0;
    double power1;
    double power2;
    double power3;


    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void setPower(double Motor0, double Motor1, double Motor2, double Motor3) {

        this.Motor0.setPower(Motor0);
        this.Motor1.setPower(Motor1);
        this.Motor2.setPower(Motor2);
        this.Motor3.setPower(Motor3);

    }

    public void setMode(boolean isBRAKE) {
        if (isBRAKE) {
            Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    public void drive(double lx,double ly, double rx, boolean turboButton) {
        motorSpeed = 1;


        power0 = motorSpeed * (ly + rx + lx);
        power1 = motorSpeed * (-ly + (rx - lx));
        power2 = motorSpeed * (-ly + rx + lx);
        power3 = motorSpeed * (ly + (rx - lx));

        setPower(power0, power1, power2, power3);
    }

    public void fieldCentric(double lx, double ly, double rx, boolean IMUReset) {
        //Motor Speed implemented soon
        double motorSpeed = 1;

        if (IMUReset) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
        double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double power0 = (rotY + rotX + rx) / denominator;
        double power3 = (rotY - rotX + rx) / denominator;
        double power1 = (rotY - rotX - rx) / denominator;
        double power2 = (rotY + rotX - rx) / denominator;

        setPower(power0, power1, power2, power3);
    }

}

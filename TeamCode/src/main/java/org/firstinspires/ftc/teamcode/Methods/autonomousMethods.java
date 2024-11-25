package org.firstinspires.ftc.teamcode.Methods;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class autonomousMethods {

    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;

    private DcMotor Motor4;

    private IMU imu;

    double Vx;
    double Vy;

    public void init(HardwareMap hardwareMap) {

        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void setDriveAngle(int angle, double speed) {

        Motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        Vx = speed * Math.sin(angle);
        Vy = speed * Math.cos(angle);

        Motor0.setPower(Vx + Vy);
        Motor1.setPower(Vx - Vy);
        Motor2.setPower(Vx + Vy);
        Motor3.setPower(Vx - Vy);
    }

    public void setAngle(int angle, double speed) {

        if (angle > 0) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= angle) {
                Motor0.setPower(1 * speed);
                Motor1.setPower(-1 * speed);
                Motor2.setPower(-1 * speed);
                Motor3.setPower(1 * speed);
            }
        } else if (angle < 0) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= angle) {
                Motor0.setPower(-1 * speed);
                Motor1.setPower(1 * speed);
                Motor2.setPower(1 * speed);
                Motor3.setPower(-1 * speed);
            }
        } else {
            Motor0.setPower(0);
            Motor1.setPower(0);
            Motor2.setPower(0);
            Motor3.setPower(0);
        }


    }

    public void stopMotors() {

        Motor0.setPower(0);
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);

        Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}

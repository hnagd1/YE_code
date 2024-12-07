package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivingMethods {

    // Creates a varible for each motor of type DcMotor, this will allows to use the motors later in the code

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bR;
    private DcMotor bL;

    // Motor Speed is not a varible that was been widely implemented in the code
    // TODO: Fix this ^^^
    private double motorSpeed;

    /* Power varibles, these will be used to set the power of each motor, the reason we do this instead of just
    setting the power in the drive Method is so that we can seprate that from the set power method which is easier to read
    and makes more sense in*/
    double powerFL;
    double powerFR;
    double powerBR;
    double powerBL;

    // sets up the imu, not much to explain here
    private IMU imu;

    public void init(HardwareMap hardwareMap) {

        // Inits all four motors, allowing us to use them later in the code
        fL = hardwareMap.get(DcMotor.class, "Motor0");
        fR = hardwareMap.get(DcMotor.class, "Motor1");
        bR = hardwareMap.get(DcMotor.class, "Motor2");
        bL = hardwareMap.get(DcMotor.class, "Motor3");

        // Sets the ZeroPowerBehavior to BRAKE which will stop the robot from drifting, this can be switched by the driven using the setMode method
        // TODO: reimplement the setMode method in robotTeleOp.java
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This sets the left motors to reverse which fixes the math stuff located in the drive method
        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sets up the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Tells the code what direction the Control Hub is facing
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        // Finalizes the IMU initialization
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void setMode(boolean isBRAKE) {
        // The robotTeleOp will pass in weather the option button is pressed, this is the isBRAKE boolean
        if (isBRAKE) {
            // If the option button is pressed, this will set the ZeroPowerBehavior to BRAKE which will stop the wheels from drifting
            fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            // If the option button is not pressed, this will set the ZeroPowerBehavior to FLOAT which will allow the wheels to drift
            // TODO: reimplement this
            fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    public void drive(double lx,double ly, double rx) {
        // Function will pass in lx (left_stick_x), ly (left_stick_y), and rx (right_stick_x)


        // Sets the Motor Speed to 1, because we need to be speedy
        motorSpeed = 1;

        // Uses math to decide the power of each motor in order to make it drive in any direction that is passed in by the lx, ly, and rx varibles
        powerFL = motorSpeed * (ly + rx - lx);
        powerFR = motorSpeed * (-ly + rx - lx);
        powerBR = motorSpeed * (-ly + rx + lx);
        powerBL = motorSpeed * (ly + rx + lx);

        // Uses these power varibles to call the setPower method which will set the power of each motor
        setPower(powerFL, powerFR, powerBR, powerBL);
    }

    public void fieldCentric(double lx, double ly, double rx, boolean IMUReset, Telemetry telemetry) {
        // Function will pass in lx (left_stick_x), ly (left_stick_y), rx (right_stick_x), and IMUReset (options button)
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        //TODO:Motor Speed implemented soon
        double motorSpeed = 1;

        // If the option button (IMUReset) is pressed, this will reset the IMU yaw angle
        if (IMUReset) {
            imu.resetYaw();
        }

        // Sets the botHeading to the IMU yaw angle in radians
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // TODO: explain this math
        double rotX = lx * Math.cos(botHeading) - ly * Math.sin(botHeading);
        double rotY = lx * Math.sin(botHeading) + ly * Math.cos(botHeading);
        telemetry.addData("lx",lx);
        telemetry.addData("ly",ly);
        telemetry.addData("rotX",rotX);
        telemetry.addData("rotY",rotY);
        telemetry.addData("rot",-botHeading);



        // Dominator is basically making sure the Motor values are all moving proportionally and not exceeding 1, I think
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double power0 = (rotY + rx - rotX) / denominator * 0.9;
        double power1 = (-rotY + rx - rotX) / denominator * 0.9;
        double power2 = (-rotY + rx + rotX) / denominator * 0.9;
        double power3 = (rotY + rx + rotX) / denominator * 0.9;

        // Uses these power varibles to call the setPower method which will set the power of each motor
        setPower(power0, power1, power2, power3);
    }

    public void setPower(double Motor0, double Motor1, double Motor2, double Motor3) {

        // Sets the power of each motor using the varibles that were passed into this method
        fL.setPower(Motor0);
        fR.setPower(Motor1);
        bR.setPower(Motor2);
        bL.setPower(Motor3);

    }

}

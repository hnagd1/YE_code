package org.firstinspires.ftc.teamcode.Methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivingMethods {

    // Creates a varible for each motor of type DcMotor, this will allows to use the motors later in the code

    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;

    // Motor Speed is not a varible that was been widely implemented in the code
    // TODO: Fix this ^^^
    private double motorSpeed;

    /* Power varibles, these will be used to set the power of each motor, the reason we do this instead of just
    setting the power in the drive Method is so that we can seprate that from the set power method which is easier to read
    and makes more sense in*/
    double power0;
    double power1;
    double power2;
    double power3;

    // sets up the imu, not much to explain here
    private IMU imu;

    public void init(HardwareMap hardwareMap) {

        // Inits all four motors, allowing us to use them later in the code
        Motor0 = hardwareMap.get(DcMotor.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");

        // Sets the ZeroPowerBehavior to BRAKE which will stop the robot from drifting, this can be switched by the driven using the setMode method
        // TODO: reimplement the setMode method in robotTeleOp.java
        Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This sets the left motors to reverse which fixes the math stuff located in the drive method
        //TODO: IF DRIVE NOT WORKING REVERSE MOTOR 0
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sets up the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Tells the code what direction the Control Hub is facing
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        // Finalizes the IMU initialization
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void setMode(boolean isBRAKE) {
        // The robotTeleOp will pass in weather the option button is pressed, this is the isBRAKE boolean
        if (isBRAKE) {
            // If the option button is pressed, this will set the ZeroPowerBehavior to BRAKE which will stop the wheels from drifting
            Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            // If the option button is not pressed, this will set the ZeroPowerBehavior to FLOAT which will allow the wheels to drift
            // TODO: reimplement this
            Motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

    }

    public void drive(double lx,double ly, double rx) {
        // Function will pass in lx (left_stick_x), ly (left_stick_y), and rx (right_stick_x)


        // Sets the Motor Speed to 1, because we need to be speedy
        motorSpeed = 1;

        // Uses math to decide the power of each motor in order to make it drive in any direction that is passed in by the lx, ly, and rx varibles
        power0 = motorSpeed * (ly + rx - lx);
        power1 = motorSpeed * (-ly + rx - lx);
        power2 = motorSpeed * (-ly + rx + lx);
        power3 = motorSpeed * (ly + rx + lx);

        // Uses these power varibles to call the setPower method which will set the power of each motor
        setPower(power0, power1, power2, power3);
    }

    public void fieldCentric(double lx, double ly, double rx, boolean IMUReset) {
        // Function will pass in lx (left_stick_x), ly (left_stick_y), rx (right_stick_x), and IMUReset (options button)

        //TODO:Motor Speed implemented soon
        double motorSpeed = 1;

        // If the option button (IMUReset) is pressed, this will reset the IMU yaw angle
        if (IMUReset) {
            imu.resetYaw();
        }

        // Sets the botHeading to the IMU yaw angle in radians
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // TODO: explain this math
        double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
        double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);

        // Deminator is basicly making sure the Motor values are all moving propetinaly and not execding 1, I think
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double power0 = (rotY + rotX + rx) / denominator;
        double power3 = (rotY - rotX + rx) / denominator;
        double power1 = (rotY - rotX - rx) / denominator;
        double power2 = (rotY + rotX - rx) / denominator;

        // Uses these power varibles to call the setPower method which will set the power of each motor
        setPower(power0, power1, power2, power3);
    }

    public void setPower(double Motor0, double Motor1, double Motor2, double Motor3) {

        // Sets the power of each motor using the varibles that were passed into this method
        this.Motor0.setPower(Motor0);
        this.Motor1.setPower(Motor1);
        this.Motor2.setPower(Motor2);
        this.Motor3.setPower(Motor3);

    }

}

package org.firstinspires.ftc.teamcode;import com.qualcomm.robotcore.eventloop.opmode.OpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import org.firstinspires.ftc.teamcode.Methods.drivingMethods;

@TeleOp
public class robotTeleOpDrive extends OpMode {

    boolean IMUReset;
    drivingMethods drive = new drivingMethods();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        robotCentricDrive();
    }

    public void robotCentricDrive() {
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        drive.drive(lx, ly, rx, gamepad1.right_trigger);
    }

    public void fieldCentricDrive() {
        IMUReset = gamepad1.options;
        double lx = gamepad1.left_stick_x * 1.1;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        drive.fieldCentric(lx, ly, rx, gamepad1.right_trigger, IMUReset, telemetry, getRuntime());
    }

    public void stop() {
        drive.setMode(true);
        drive.setPower(0,0,0,0);
    }
}



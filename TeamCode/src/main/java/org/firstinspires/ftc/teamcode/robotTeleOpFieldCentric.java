package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Methods.attachmentMethods;
import org.firstinspires.ftc.teamcode.Methods.autonomousMethods;
import org.firstinspires.ftc.teamcode.Methods.drivingMethods;

//@TeleOp
public class robotTeleOpFieldCentric extends OpMode {

    boolean brakeButton = false;
    boolean rSB = false;

    drivingMethods drive = new drivingMethods();
    autonomousMethods auto = new autonomousMethods();
    attachmentMethods attachment = new attachmentMethods();
    LinearSlide ls = new LinearSlide();

    @Override
    public void init() {
        drive.init(hardwareMap);
        auto.init(hardwareMap);
        attachment.init(hardwareMap);
        telemetry.addData("Version", 1);
        ls.init(hardwareMap);
    }

    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x * 1.1; //Strafe variable
        double ly = -gamepad1.left_stick_y; //Forward and backward
        double rx = gamepad1.right_stick_x; //Turn right and left
        boolean IMUReset = gamepad1.options;

        if(gamepad1.right_stick_button && !rSB) {
            brakeButton = !brakeButton;
        }

        if (gamepad2.b) {
            // auto.setAngle(90, 25,3000);
            attachment.toggleIntake(1);
        } else if (gamepad2.y) {
            attachment.toggleIntake(0);
        } else if (gamepad2.x) {
            attachment.toggleIntake(-1);
        }

        rSB = gamepad1.right_stick_button;
        telemetry.addData("Brake toggle", brakeButton);


        //TODO: Rename variables so they accurate, play around with spin slow bug
        drive.fieldCentric(lx, ly, rx, IMUReset);
        drive.setMode(brakeButton);
        if (IMUReset) {
            telemetry.addLine("Updated ori");
        }

        double ry2 = -gamepad2.right_stick_y;
        attachment.toggleLiftArm(ry2, telemetry);
        if (gamepad2.a) {
            ls.basketServo(gamepad2.a);
        }


        ls.loop(gamepad1.left_bumper, gamepad1.right_bumper, true, telemetry);
    }

    public void stop() {
        attachment.resetServo();
    }
}

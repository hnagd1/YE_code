package org.firstinspires.ftc.teamcode.Odometry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class RoadRunnerTest extends LinearOpMode {
    public static class Ls {
        private DcMotor ls0;
        private DcMotor ls1;

        public Ls(HardwareMap hardwareMap) {
            ls0 = hardwareMap.get(DcMotor.class, "Motor5");
            ls1 = hardwareMap.get(DcMotor.class, "Motor6");
            ls0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ls1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ls0.setTargetPosition(0);
            ls1.setTargetPosition(0);
            ls0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ls0.setDirection(DcMotorSimple.Direction.FORWARD);
            ls1.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public Action retract() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    ls0.setTargetPosition(0);
                    ls1.setTargetPosition(0);
                    ls0.setPower(0.7);
                    ls1.setPower(0.7);
                    if (ls0.getCurrentPosition() < 10) {
                        ls0.setPower(0);
                        ls1.setPower(0);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }

        public Action extend(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    ls0.setTargetPosition(3100);
                    ls1.setTargetPosition(3100);
                    ls0.setPower(0.7);
                    ls1.setPower(0.7);
                    if (ls0.getCurrentPosition() > 3050) {
                        ls0.setPower(0);
                        ls1.setPower(0);
                        return false;
                    } else {
                        return true;
                    }
                }
            };
        }

    }
    public static class Bucket {
        private Servo servo;

        public Bucket (HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "Servo2");
        }

        public Action ready(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.935);
                    return false;
                }
            };
        }

        public Action dump(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.75);
                    return false;
                }
            };
        }
    }



    @Override
    public void runOpMode() {
        Ls ls = new Ls(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);

        Action test;
        test = new SequentialAction(
                ls.extend(),
                ls.retract());
        //TODO: Figure out stop code here

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        test
                )
        );
    }
}

//TODO: WHEN CODING USE THIS TEMPLATE
/*
public Action [INSERT NAME](){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    return false;
                }
            };
        }
 */
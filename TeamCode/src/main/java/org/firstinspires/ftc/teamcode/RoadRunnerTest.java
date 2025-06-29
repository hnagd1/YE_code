package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

    public static class JointServ {
        private Servo servo;

        public JointServ(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "Servo1");
        }

        public Action dump() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.66);
                    return false;
                }
            };
        }

        public Action intake() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0);
                    //TODO: find the position we actually want
                    return false;
                }
            };
        }

        public Action moveOutOfWay(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    servo.setPosition(0.075);
                    return false;
                }
            };
        }
    }

    public static class ArmMot {
        private DcMotor motor;

        public ArmMot(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotor.class, "Motor 4");
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public Action dump() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setTargetPosition(-2500);
                    if (-2450 > motor.getCurrentPosition() && motor.getCurrentPosition() > -2550) {
                        motor.setPower(0);
                        return false;
                    } else {
                        motor.setPower(1);
                        return true;
                    }
                }
            };
        }

        public Action intake() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    return false;
                }
            };

        }
        public Action moveOutOfWay(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    motor.setTargetPosition(-3800);
                    if (-3750 > motor.getCurrentPosition() && motor.getCurrentPosition() > -3850) {
                        motor.setPower(0);
                        return false;
                    } else {
                        motor.setPower(1);
                        return true;
                    }
                }
            };
        }
    }

    //TODO: Class code template:
    /*
    public static class [NAME] {
    }
    */

    //TODO: Action code examples
    /*
public Action dump(){
    return new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    };
}
     */

    /**
     * Custom actions will repeat until false is returned.
     * For servos, just return false and add in sleep later
     */
    @Override
    public void runOpMode() {
        Ls ls = new Ls(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);
        JointServ jointServ = new JointServ(hardwareMap);
        ArmMot armMot = new ArmMot(hardwareMap);

        //This is where we setup actions that require sleep
        Action lsdump =
                new SequentialAction(
                        bucket.dump(),
                        new SleepAction(0.8)
                );

        Action intakeDump =
                new ParallelAction(
                        new SequentialAction(
                            jointServ.dump(),
                            new SleepAction(1)
                        ),
                        armMot.dump()
                );


        //TODO: Figure out stop code here

        waitForStart();
        /*
        AFTER START
         */

        Actions.runBlocking(
                new SequentialAction(
                        ls.extend(),
                        lsdump,
                        ls.retract()
                )
        );
    }
}

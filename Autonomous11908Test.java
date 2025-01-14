package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

enum Direction {
    FORWARD,
    BACKWARD,
    UP,
    DOWN,
}

@Autonomous(name="Shift: Test Auto", group="Robot")
public class Autonomous11908Test extends LinearOpMode {
    private DcMotor ArmRotation;
    private Servo ClawGrabber;
    private DcMotor ClawWrist;
    private Blinker ControlHub;
    private Blinker ExpansionHub;
    private DcMotor LinearHexMotor;
    private DcMotor back_left_drive;
    private DcMotor back_right_drive;
    private DcMotor front_left_drive;
    private DcMotor front_right_drive;
    
    static final double COUNTS_PER_CM = (560/(7.5*Math.PI)); // COUNTS_PER_REV / (WHEEL_DIAMETER_CM*PI)
    
    @Override
    public void runOpMode() {
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        ArmRotation = hardwareMap.get(DcMotor.class, "Arm Rotation");
        ClawWrist = hardwareMap.get(DcMotor.class, "Claw Wrist");
        LinearHexMotor = hardwareMap.get(DcMotor.class, "Linear Hex Motor");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
        ClawGrabber = hardwareMap.get(Servo.class, "Claw Grabber");
        
        front_left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
        
        ArmRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClawWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        //move(10.0, Direction.FORWARD, 0.7);
        //move(10.0, Direction.LEFT, 0.7);-
        //move(10.0, Direction.BACKWARD, 0.7);
        //move(10.0, Direction.RIGHT, 0.7);
        //rotate(180, 0.7);
        //move(100.0, Direction.FORWARD, 0.7);
        //rotate(-90.0, 1.0);
        armrotation(20.0);
        
    }
    
    public void armrotation(double distance) {
        ArmRotation.setTargetPosition((int)(distance * 10));
        ArmRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void move(double distance, Direction direction, double timeout) {
        int front_left_distance = (int)-(distance * COUNTS_PER_CM);
        int front_right_distance = (int)-(distance * COUNTS_PER_CM);
        int back_left_distance = (int)-(distance * COUNTS_PER_CM);
        int back_right_distance = (int)-(distance * COUNTS_PER_CM);
        String direction_string = "UNKNOWN";
        switch (direction) {
            case FORWARD: {
                front_left_distance = (int)-(distance * COUNTS_PER_CM);
                front_right_distance = (int)-(distance * COUNTS_PER_CM);
                back_left_distance = (int)-(distance * COUNTS_PER_CM);
                back_right_distance = (int)-(distance * COUNTS_PER_CM);
                direction_string = "FORWARD";
                break;
            }
            case BACKWARD: {
                front_left_distance = (int)(distance * COUNTS_PER_CM);
                front_right_distance = (int)(distance * COUNTS_PER_CM);
                back_left_distance = (int)(distance * COUNTS_PER_CM);
                back_right_distance = (int)(distance * COUNTS_PER_CM);
                direction_string = "BACKWARD";
                break;
            }
            
        }
        
        int front_left_goal = (front_left_drive.getCurrentPosition() + front_left_distance);
        int front_right_goal = (front_right_drive.getCurrentPosition() + front_right_distance);
        int back_left_goal = (back_left_drive.getCurrentPosition() + back_left_distance);
        int back_right_goal = (back_right_drive.getCurrentPosition() + back_right_distance);
        telemetry.addData("Going to",  "%7d %7d %7d %7d",
            front_left_goal,
            front_right_goal,
            back_left_goal,
            back_right_goal
        );
        
        telemetry.addData("Direction: ", direction_string);
        front_left_drive.setTargetPosition(front_left_goal);
        front_right_drive.setTargetPosition(front_right_goal);
        back_left_drive.setTargetPosition(back_left_goal);
        back_right_drive.setTargetPosition(back_right_goal);
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        front_left_drive.setPower(0.2);
        front_right_drive.setPower(0.2);
        back_left_drive.setPower(0.76);
        back_right_drive.setPower(0.76);
        
        while (
            opModeIsActive() &&
            front_left_drive.isBusy() &&
            front_right_drive.isBusy() &&
            back_left_drive.isBusy() &&
            back_right_drive.isBusy()
        ) {
                telemetry.addData("Going to", "%7d %7d %7d %7d",
                    front_left_goal,
                    front_right_goal,
                    back_left_goal,
                    back_right_goal
                );
                telemetry.addData("Direction: ", direction_string);
                telemetry.addData("Front Left Running to", " %7d :%7d", front_left_goal, front_left_drive.getCurrentPosition());
                telemetry.addData("Front Right Running to", " %7d :%7d", front_right_goal, front_right_drive.getCurrentPosition());
                telemetry.addData("Back Left Running to", " %7d :%7d", back_left_goal, back_left_drive.getCurrentPosition());
                telemetry.addData("Back Right Running to", " %7d :%7d", back_right_goal, back_right_drive.getCurrentPosition());
                telemetry.update();
            }
        front_left_drive.setPower(0);
        front_right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep((long)(timeout*1000));
    }
    
    public void rotate(double degrees, double timeout) {
        degrees = degrees/2;
        int front_left_distance = (int)-(degrees * COUNTS_PER_CM);
        int front_right_distance = (int)(degrees * COUNTS_PER_CM);
        int back_left_distance = (int)-(degrees * COUNTS_PER_CM);
        int back_right_distance = (int)(degrees * COUNTS_PER_CM);
        String direction_string = "UNKNOWN";
        if (degrees > 0) {
            direction_string = "RIGHT";
        } else if (degrees < 0) {
            direction_string = "LEFT";
            degrees = -(degrees);
        }
        int front_left_goal = (front_left_drive.getCurrentPosition() + front_left_distance);
        int front_right_goal = (front_right_drive.getCurrentPosition() + front_right_distance);
        int back_left_goal = (back_left_drive.getCurrentPosition() + back_left_distance);
        int back_right_goal = (back_right_drive.getCurrentPosition() + back_right_distance);
        telemetry.addData("Going to",  "%7d %7d %7d %7d",
            front_left_goal,
            front_right_goal,
            back_left_goal,
            back_right_goal
        );
        telemetry.addData("Direction: ", direction_string);
        front_left_drive.setTargetPosition(front_left_goal);
        front_right_drive.setTargetPosition(front_right_goal);
        back_left_drive.setTargetPosition(back_left_goal);
        back_right_drive.setTargetPosition(back_right_goal);
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        front_left_drive.setPower(0.2);
        front_right_drive.setPower(0.2);
        back_left_drive.setPower(0.76);
        back_right_drive.setPower(0.76);
        
        while (
            opModeIsActive() &&
            front_left_drive.isBusy() &&
            front_right_drive.isBusy() &&
            back_left_drive.isBusy() &&
            back_right_drive.isBusy()
        ) {
                telemetry.addData("Going to", "%7d %7d %7d %7d",
                    front_left_goal,
                    front_right_goal,
                    back_left_goal,
                    back_right_goal
                );
                telemetry.addData("Direction: ", direction_string);
                telemetry.addData("Front Left Running to", " %7d :%7d", front_left_goal, front_left_drive.getCurrentPosition());
                telemetry.addData("Front Right Running to", " %7d :%7d", front_right_goal, front_right_drive.getCurrentPosition());
                telemetry.addData("Back Left Running to", " %7d :%7d", back_left_goal, back_left_drive.getCurrentPosition());
                telemetry.addData("Back Right Running to", " %7d :%7d", back_right_goal, back_right_drive.getCurrentPosition());
                telemetry.update();
            }
        front_left_drive.setPower(0);
        front_right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep((long)(timeout*1000));
    }
}

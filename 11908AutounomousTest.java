package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;

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
    
    static final double COUNTS_PER_CM = (28/(7.5*Math.PI)); // COUNTS_PER_REV / (WHEEL_DIAMETER_CM*PI)
    
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
        waitForStart();
        move(20.0, 20, 1.0);
    }
    
    public void move(double distance, int direction, double timeout) {
        front_left_drive.setTargetPosition(front_left_drive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM));
        front_right_drive.setTargetPosition(front_right_drive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM));
        back_left_drive.setTargetPosition(back_left_drive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM));
        back_right_drive.setTargetPosition(back_right_drive.getCurrentPosition() + (int)(distance * COUNTS_PER_CM));
        
        front_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        front_left_drive.setPower(0.3);
        front_right_drive.setPower(0.3);
        back_left_drive.setPower(0.3);
        back_right_drive.setPower(0.3);
    }
}

package org.firstinspires.ftc.teamcode.UniversalCode;


import static java.lang.Math.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Four Wheel drive class for robot. Takes Four motors, named:
 * front_left_drive,
 * back_left_drive,
 * front_right_drive,
 * back_right_drive
 *
 * call Init in initialization, and update in the update loop.
 */
public class fourWheelDrive {
    private DcMotor front_left_drive;
    private DcMotor back_left_drive;
    private DcMotor front_right_drive;
    private DcMotor back_right_drive;
    double forwardsVelocity;
    double horizontalVelocity;
    double Dir_x, Dir_y;
    double speed = 0.3;

    /**
     * Initailization.
     * @param hardwareMap the robot hardware map
     */
    public void Init(@NonNull com.qualcomm.robotcore.hardware.HardwareMap hardwareMap){
        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");


        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        //front_left_drive.setDirection(DcMotor.Direction.REVERSE);
        front_right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
    }
    private double Clamp(double value, double min, double max) {
        return Math.min(max, Math.max(value, min));
    }


    /**
     * Update loop for Four Wheel Drive
     * @param telemetry It's the telemetry...
     * @param gamepad1 It's Gamepad 1, idk
     * @param gamepad2 It's Gamepad 2, I guess...
     */
    public void Update(@NonNull org.firstinspires.ftc.robotcore.external.Telemetry telemetry, @NonNull com.qualcomm.robotcore.hardware.Gamepad gamepad1,
                       com.qualcomm.robotcore.hardware.Gamepad gamepad2) {
        // Mario Cart Controls
        forwardsVelocity = -gamepad1.left_trigger + gamepad1.right_trigger;
        horizontalVelocity = gamepad1.left_stick_x;
        // Right Stick
        Dir_x = gamepad1.right_stick_x;
        forwardsVelocity -= gamepad1.right_stick_y;

        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        front_left_drive.setPower(Clamp((((forwardsVelocity + Dir_x) * speed) + horizontalVelocity), -1.0, 1.0));
        front_right_drive.setPower(Clamp(((forwardsVelocity - Dir_x) * speed) - horizontalVelocity, -1.0, 1.0));

        back_left_drive.setPower(Clamp(((forwardsVelocity - Dir_x) * speed) + horizontalVelocity, -1.0, 1.0));
        back_right_drive.setPower(Clamp(((forwardsVelocity + Dir_x )* speed) - horizontalVelocity, -1.0, 1.0));
        // Telemetry is how we send data to the driverhub
        telemetry.addData("Power", forwardsVelocity);
        telemetry.addData("direction", horizontalVelocity);
    }

}

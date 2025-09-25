package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "_11908Omnidrive (Blocks to Java)")
public class _11908Omnidrive extends LinearOpMode {

  private DcMotor front_left_drive;
  private DcMotor back_left_drive;
  private DcMotor ArmRotation;
  private DcMotor ClawWrist;
  private DcMotor LinearHexMotor;
  private DcMotor front_right_drive;
  private DcMotor back_right_drive;
  private Servo ClawGrabber;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double Speed;

    front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
    back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
    ArmRotation = hardwareMap.get(DcMotor.class, "Arm Rotation");
    ClawWrist = hardwareMap.get(DcMotor.class, "Claw Wrist");
    LinearHexMotor = hardwareMap.get(DcMotor.class, "Linear Hex Motor");
    front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
    back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");
    ClawGrabber = hardwareMap.get(Servo.class, "Claw Grabber");

    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    front_left_drive.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    back_left_drive.setDirection(DcMotor.Direction.REVERSE);
    ArmRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ClawWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LinearHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Speed = 0.3;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad1.b) {
          Speed = 0;
        } else if (gamepad1.left_trigger > 0.3) {
          Speed = 1;
        } else if (gamepad1.right_trigger > 0.3) {
          Speed = 0.2;
        } else if (gamepad1.left_bumper) {
          Speed = 0.75;
        } else if (gamepad1.right_bumper) {
          Speed = 0.35;
        } else {
          Speed = 0.5;
        }
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        front_left_drive.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * Speed);
        front_right_drive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * Speed);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        back_left_drive.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * Speed);
        back_right_drive.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * Speed);
        if (gamepad2.dpad_down) {
          LinearHexMotor.setPower(1);
        } else if (gamepad2.dpad_up) {
          LinearHexMotor.setPower(-1);
        } else {
          LinearHexMotor.setPower(0);
        }
        ClawWrist.setPower(gamepad2.left_stick_y * 0.3);
        ArmRotation.setPower(gamepad2.right_stick_y * 0.7);
        if (gamepad2.right_stick_y == 0) {
          if (!(ArmRotation.getCurrentPosition() <= -1230)) {
            ArmRotation.setPower(- 0.21);
          }
        }
        if (gamepad2.a) {
          ClawGrabber.setPosition(-0.9);
        }
        if (gamepad2.b) {
          ClawGrabber.setPosition(0.9);
        }
        // Telemetry is how we send data to the driverhub
        telemetry.addData("Power", gamepad2.right_stick_y);
        telemetry.addData("Speed", Speed);
        telemetry.addData("Claw Position", ClawGrabber.getPosition());
        telemetry.addData("Arm Rotation Position", ArmRotation.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
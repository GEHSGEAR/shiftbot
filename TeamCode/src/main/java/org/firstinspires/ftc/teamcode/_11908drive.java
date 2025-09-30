package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.UniversalCode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "11908Omnidrivee", group="Linear OpMode")
public class _11908drive extends LinearOpMode {

fourWheelDrive drive;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        drive = new fourWheelDrive();
        drive.Init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            while (opModeIsActive()) {
                drive.Update(telemetry, gamepad1, gamepad2);

                telemetry.update();
            }
        }
    }
}



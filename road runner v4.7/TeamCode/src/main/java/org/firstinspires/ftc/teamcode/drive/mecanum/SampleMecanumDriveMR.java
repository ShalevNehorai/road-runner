package org.firstinspires.ftc.teamcode.drive.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToCM;

/*
 * Simple mecanum drive hardware implementation for Modern Robotics hardware.
 */
public class SampleMecanumDriveMR extends SampleMecanumDriveBase {
    /*
     * As you may know, the MR communication system is implemented asynchronously. Thus, all
     * hardware calls return immediately (reads are cached and writes are queued). To ensure that
     * Road Runner isn't needlessly running on stale data (this is actually harmful), we delay after
     * each call to setMotorPowers() with the hope that, in most cases, new data will be available
     * by the next iteration (though this can never be guaranteed). Although it may seem attractive
     * to decrease this number and increase your control loop frequency, do so at your own risk.
     */
    private static final int MOTOR_WRITE_DELAY = 20;

    private DcMotor frontLeftDrive, backLeftDrive, backRightDrive, frontRightDrive;
    private List<DcMotor> motors;
    private BNO055IMU imu;

    public SampleMecanumDriveMR(HardwareMap hardwareMap) {
        super();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");

        motors = Arrays.asList(frontLeftDrive, backLeftDrive, backRightDrive, frontRightDrive);

        for (DcMotor motor : motors) {
            motor.setMode(RUN_USING_ENCODER ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) frontLeftDrive.getController();
        DifferentialControlLoopCoefficients coefficients = controller.getDifferentialControlLoopCoefficients(frontLeftDrive.getPortNumber());
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotor motor : motors) {
            ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) motor.getController();
            controller.setDifferentialControlLoopCoefficients(motor.getPortNumber(), new DifferentialControlLoopCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            wheelPositions.add(encoderTicksToCM(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeftDrive.setPower(v);
        backLeftDrive.setPower(v1);
        backRightDrive.setPower(v2);
        frontRightDrive.setPower(v3);

        try {
            Thread.sleep(MOTOR_WRITE_DELAY);
        } catch (InterruptedException e) {
            // do nothing
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}

package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

@Config(value = "DEVICE_TEST")
@TeleOp(group = "tests")
public class DeviceTest extends LinearOpMode {
    public static String DEVICE_NAME = "";

    public enum DeviceType {
        DC_MOTOR, DIGITAL_CHANNEL, ANALOG_INPUT, SERVO, GYRO, BATTERY_VOLTAGE, COLOR_SENSOR, DISTANCE_SENSOR, PINPOINT, NONE
    }

    private DeviceType getDeviceClass(HardwareDevice hardwareDevice) {
        if (hardwareDevice instanceof DcMotorEx) return DeviceType.DC_MOTOR;
        if (hardwareDevice instanceof DigitalChannel) return DeviceType.DIGITAL_CHANNEL;
        if (hardwareDevice instanceof VoltageSensor) return DeviceType.BATTERY_VOLTAGE;
        if (hardwareDevice instanceof AnalogInput) return DeviceType.ANALOG_INPUT;
        if (hardwareDevice instanceof Servo) return DeviceType.SERVO;
        if (hardwareDevice instanceof IMU) return DeviceType.GYRO;
        if (hardwareDevice instanceof ColorSensor) return DeviceType.COLOR_SENSOR;
        if (hardwareDevice instanceof DistanceSensor) return DeviceType.DISTANCE_SENSOR;
        if (hardwareDevice instanceof GoBildaPinpointDriver) return DeviceType.PINPOINT;
        return DeviceType.NONE;
    }

    public static double VALUE_TO_SEND = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardwareMap.getAll(IMU.class).forEach(imu -> imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(new Quaternion()))));
        hardwareMap.getAll(GoBildaPinpointDriver.class).forEach(GoBildaPinpointDriver::resetPosAndIMU);

        Object[] deviceNames = hardwareMap.getAllNames(HardwareDevice.class).toArray();

        telemetry.addLine("All devices:");
        telemetry.addLine(Arrays.toString(deviceNames));

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            try {
                HardwareDevice hardwareDevice = hardwareMap.get(DEVICE_NAME);

                DeviceType _deviceType = getDeviceClass(hardwareDevice);

                telemetry.addData("Device type", _deviceType);
                telemetry.addLine(hardwareDevice.getConnectionInfo());

                switch (_deviceType) {
                    case DC_MOTOR:
                        DcMotorEx motor = (DcMotorEx) hardwareDevice;
                        motor.setDirection(DcMotorSimple.Direction.FORWARD);
                        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        telemetry.addData("encoder pos", motor.getCurrentPosition());
                        telemetry.addData("encoder vel", motor.getVelocity());
                        telemetry.addData("motor current", motor.getCurrent(CurrentUnit.AMPS));
                        motor.setPower(VALUE_TO_SEND);
                        break;

                    case SERVO:
                        Servo servo = (Servo) hardwareDevice;
                        servo.setPosition(VALUE_TO_SEND);
                        break;

                    case DIGITAL_CHANNEL:
                        DigitalChannel digitalChannel = (DigitalChannel) hardwareDevice;
                        digitalChannel.setMode(DigitalChannel.Mode.INPUT);
                        telemetry.addData("State", digitalChannel.getState());
                        break;

                    case ANALOG_INPUT:
                        AnalogInput analogInput = (AnalogInput) hardwareDevice;
                        telemetry.addData("Voltage", analogInput.getVoltage());
                        break;

                    case GYRO:
                        IMU imu = (IMU) hardwareDevice;
                        YawPitchRollAngles ypra = imu.getRobotYawPitchRollAngles();
                        telemetry.addLine("NOTE: REV IMU orietnation may be off");
                        telemetry.addLine("Angle units are Degrees");
                        telemetry.addData("yaw", ypra.getYaw(DEGREES));
                        telemetry.addData("pitch", ypra.getPitch(DEGREES));
                        telemetry.addData("roll", ypra.getRoll(DEGREES));
                        break;

                    case BATTERY_VOLTAGE:
                        VoltageSensor voltageSensor = (VoltageSensor) hardwareDevice;
                        telemetry.addData("Voltage", voltageSensor.getVoltage());
                        break;

                    case COLOR_SENSOR:
                        ColorSensor colorSensor = (ColorSensor) hardwareDevice;
                        telemetry.addData("red", colorSensor.red());
                        telemetry.addData("green", colorSensor.green());
                        telemetry.addData("blue", colorSensor.blue());
                        telemetry.addData("alpha", colorSensor.alpha());
                        break;

                    case DISTANCE_SENSOR:
                        DistanceSensor distanceSensor = (DistanceSensor) hardwareDevice;
                        telemetry.addData("range (cm)", distanceSensor.getDistance(DistanceUnit.CM));
                        break;

                    case PINPOINT:
                        GoBildaPinpointDriver pinpoint = (GoBildaPinpointDriver) hardwareDevice;

                        telemetry.addData("hz", pinpoint.getFrequency());
                        telemetry.addData("x odometr", pinpoint.getEncoderX());
                        telemetry.addData("y odometr", pinpoint.getEncoderY());
                        telemetry.addData("angle", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));

                        break;

                    case NONE:

                    default:
                        break;
                }
            } catch (ClassCastException e) {
                telemetry.addLine("ERR: wrong device type selected");
            } catch (IllegalArgumentException e) {
                telemetry.addLine("Device not found");
            }

            telemetry.addLine("All devices:");
            telemetry.addLine(Arrays.toString(deviceNames));

            telemetry.update();
        }
    }
}

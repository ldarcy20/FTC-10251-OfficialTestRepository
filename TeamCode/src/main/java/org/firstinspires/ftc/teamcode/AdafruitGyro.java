package org.firstinspires.ftc.teamcode;


        //import com.qualcomm.ftcrobotcontroller.opmodes.FtcOpModeRegister;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode;
        import com.qualcomm.robotcore.util.Range;

/**
 * This is an Adafruit Bosch BNO055 Inertial Measurement Unit.
 * It gets data in quaternions from the fused sensor data, and returns yaw, pitch, and roll.
 */

public class AdafruitGyro {

    //private final I2cDevice imu;
    private final I2cDeviceSynch imuManager;

    // default I2C address is 0x28
    //private static final int I2C_ADDRESS = 0x28;
    private static final I2cAddr I2C_ADDRESS = new I2cAddr(0x28);

    // register page set to page 0 (0 or 1)
    // makes SYS_TRIGGER_REGISTER visible
    private static final int PAGE_ID_REGISTER = 0x07;
    private static final int PAGE_ID_VALUE = 0x00;

    // 7th bit: use internal oscillator (0) or external connected crystal on Adafruit board (1)
    // 6th bit: set to reset all interrupt status bits
    // 5th bit: set to reset system
    // 0th bit: set to trigger self-test (done when powered on, POST)
    // setting to 0xE0 (11100000b) to reset everything, self-test already performed at startup
    private static final int SYS_TRIGGER_REGISTER = 0x3F;
    private static final int SYS_TRIGGER_VALUE = 0xE0;

    // selects output units
    // magnetic field strength in microTesla
    // quaternions in quaternion units
    // 0th bit (acceleration units): m/s^2 (0) or mg (1)
    // 1st bit (angular rate): dps (0) or rps (1)
    // 2nd bit (euler angles): degrees (0) or radians (1)
    // 4th bit (temperature): Celsius (0) or Fahrenheit (1)
    // 7th bit (rotation angle conventions): Windows (0) or Android (1)
    //      pitch (Android): +180 to -180 (turning clockwise decreases values)
    //      pitch (Windows): -180 to +180 (turning clockwise increases values)
    //      roll: -90 to 90 (increasing with increasing inclination
    //      heading/yaw: 0 to 360 (turning clockwise increases values)
    // set to 0x80 (10000000b) to get Android pitch convention, stay in metric
    private static final int UNIT_SEL_REGISTER = 0x3B;
    private static final int UNIT_SEL_VALUE = 0x80;

    // sets the operation mode
    // starts into CONFIG_MODE by default, which is only mode that allows configuration changes
    // IMU uses only the accelerometers and gyroscopes
    private static final int OPR_MODE_REGISTER = 0x3D;
    private static final int OPR_MODE_IMU = 0x08;

    // [CALIB_STAT] 0x35 register addresss: is it useful?
    // [ST_RESULT] 0x36 register address: is it useful?
    // [SYS_STATUS] 0x39 register address: is it useful?
    // https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
    // http://ftcforum.usfirst.org/showthread.php?5774-BNO055-IMU-not-turning-90-degrees-consistently/page2

    public static final int
            BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
            BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
            BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
            BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
            BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
            BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
            BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
            BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27;

    public static final int
            BNO055_QUATERNION_DATA_W_LSB_CACHE = 0,
            BNO055_QUATERNION_DATA_W_MSB_CACHE = 1,
            BNO055_QUATERNION_DATA_X_LSB_CACHE = 2,
            BNO055_QUATERNION_DATA_X_MSB_CACHE = 3,
            BNO055_QUATERNION_DATA_Y_LSB_CACHE = 4,
            BNO055_QUATERNION_DATA_Y_MSB_CACHE = 5,
            BNO055_QUATERNION_DATA_Z_LSB_CACHE = 6,
            BNO055_QUATERNION_DATA_Z_MSB_CACHE = 7;

    private static final int
            READ_WINDOW_START = BNO055_QUATERNION_DATA_W_LSB_ADDR,
            READ_WINDOW_LENGTH = 8;

    private static final int N_OFFSET_READINGS = 50;
    private static final double OFFSET_ERROR = 1E-6;
    private boolean offsetsInitialized = false;
    private int nOffset = 0;
    private double[]
            yawOffset = new double[N_OFFSET_READINGS + 1],
            pitchOffset = new double[N_OFFSET_READINGS + 1],
            rollOffset = new double[N_OFFSET_READINGS + 1];
    private double[]
            quaternionVector = new double[5],
            yprAngles = new double[3];


    public AdafruitGyro(I2cDevice imu) {
        imuManager = new I2cDeviceSynchImpl(imu, I2C_ADDRESS, false);
        imuManager.engage();
        imuManager.write8(PAGE_ID_REGISTER, PAGE_ID_VALUE);
        imuManager.write8(SYS_TRIGGER_REGISTER, SYS_TRIGGER_VALUE);
        imuManager.write8(UNIT_SEL_REGISTER, UNIT_SEL_VALUE);
        imuManager.write8(OPR_MODE_REGISTER, OPR_MODE_IMU);
        imuManager.setReadWindow(new ReadWindow(READ_WINDOW_START, READ_WINDOW_LENGTH, ReadMode.REPEAT));
    }


    public double[] getAngles() {

        double yaw, pitch, roll;
        double tempQuatYaw, tempQuatPitch, tempQuatRoll;
        double q0, q1, q2, q3;
        byte[] imuCache = imuManager.read(READ_WINDOW_START, READ_WINDOW_LENGTH);

        quaternionVector[0] =  //Quaternion component "W"
                (double) ((short)
                        ((imuCache[BNO055_QUATERNION_DATA_W_MSB_CACHE] & 0XFF) << 8)
                        | (imuCache[BNO055_QUATERNION_DATA_W_LSB_CACHE] & 0XFF)) / 16384.0;
        quaternionVector[1] =  //Quaternion component "X"
                (double) ((short)
                        ((imuCache[BNO055_QUATERNION_DATA_X_MSB_CACHE] & 0XFF) << 8)
                        | (imuCache[BNO055_QUATERNION_DATA_X_LSB_CACHE] & 0XFF)) / 16384.0;
        quaternionVector[2] =  //Quaternion component "Y"
                (double) ((short)
                        ((imuCache[BNO055_QUATERNION_DATA_Y_MSB_CACHE] & 0XFF) << 8)
                        | (imuCache[BNO055_QUATERNION_DATA_Y_LSB_CACHE] & 0XFF)) / 16384.0;
        quaternionVector[3] =  //Quaternion component "Z"
                (double) ((short)
                        ((imuCache[BNO055_QUATERNION_DATA_Z_MSB_CACHE] & 0XFF) << 8)
                        | (imuCache[BNO055_QUATERNION_DATA_Z_LSB_CACHE] & 0XFF)) / 16384.0;

        quaternionVector[4] = Math.pow(quaternionVector[0], 2.0) + Math.pow(quaternionVector[1], 2.0)
                + Math.pow(quaternionVector[2], 2.0)
                + Math.pow(quaternionVector[3], 2.0);

        q0 = quaternionVector[0];
        q1 = quaternionVector[1];
        q2 = quaternionVector[2];
        q3 = quaternionVector[3];

        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        tempQuatPitch = 2.0 * (q0 * q2 - q3 * q1);
        tempQuatPitch = Range.clip(tempQuatPitch, -1.0, 1.0);
        tempQuatPitch = Math.asin(tempQuatPitch) * 180.0 / Math.PI;

        tempQuatYaw = Math.atan2( 2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;

        tempQuatRoll = Math.atan2( 2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * 180.0 / Math.PI;

        if (!offsetsInitialized) {
            yawOffset[nOffset] = tempQuatYaw;
            pitchOffset[nOffset] = tempQuatPitch;
            rollOffset[nOffset] = tempQuatRoll;
            nOffset++;

            if (nOffset > N_OFFSET_READINGS) {
                if ( yawOffset[N_OFFSET_READINGS] - yawOffset[N_OFFSET_READINGS / 2] < OFFSET_ERROR
                        && yawOffset[N_OFFSET_READINGS / 2] - yawOffset[0] < OFFSET_ERROR
                        && pitchOffset[N_OFFSET_READINGS] - pitchOffset[N_OFFSET_READINGS / 2] < OFFSET_ERROR
                        && pitchOffset[N_OFFSET_READINGS / 2] - pitchOffset[0] < OFFSET_ERROR
                        && rollOffset[N_OFFSET_READINGS] - rollOffset[N_OFFSET_READINGS / 2] < OFFSET_ERROR
                        && rollOffset[N_OFFSET_READINGS / 2] - rollOffset[0] < OFFSET_ERROR) {
                    offsetsInitialized = true;
                } else {
                    nOffset = 0;
                }
            }

        }


        // Output yaw(heading) angles are offset-corrected and range-limited to -180 through +180
        yaw = tempQuatYaw - yawOffset[N_OFFSET_READINGS];
        yaw = (yaw >= 180.0) ? (yaw - 360.0) : ((yaw < -180.0) ? (yaw + 360.0) : yaw);
        // Output pitch angles are offset-corrected and range-limited to -90 through +90
        pitch = tempQuatPitch - pitchOffset[N_OFFSET_READINGS];
        pitch = (pitch > 90.0) ? 90.0 : ((pitch < -90.0) ? -90.0 : pitch);
        // Output roll angles are offset-corrected and range-limited to -90 through +90
        roll = tempQuatRoll - pitchOffset[N_OFFSET_READINGS];
        roll = (roll > 90.0) ? 90.0 : ((roll < -90.0) ? -90.0 : roll);

        if (!offsetsInitialized) {
            yaw = 9001;
            pitch = 9001;
            roll = 9001;
        }

        yprAngles[0] = yaw;
        yprAngles[1] = pitch;
        yprAngles[2] = roll;

        return yprAngles;

    }


}
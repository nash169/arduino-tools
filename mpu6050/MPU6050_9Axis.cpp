#include "MPU6050_9Axis.h"

MPU6050_9::MPU6050_9(uint8_t address){
  devAddr = address;
}

MPU6050_9::MPU6050_9() {
  devAddr = MPU6050_DEFAULT_ADDRESS;
}

uint8_t MPU6050_9::dmpInitialize() {
    // reset device
    DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
    reset();
    delay(30); // wait after reset

    // disable sleep mode
    DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU product ID
    DEBUG_PRINTLN(F("Getting product ID..."));
    //uint8_t productID = 0; //getProductID();
    DEBUG_PRINT(F("Product ID = "));
    DEBUG_PRINT(productID);

    // get MPU hardware revision
    DEBUG_PRINTLN(F("Selecting user bank 16..."));
    setMemoryBank(0x10, true, true);
    DEBUG_PRINTLN(F("Selecting memory byte 6..."));
    setMemoryStartAddress(0x06);
    DEBUG_PRINTLN(F("Checking hardware revision..."));
    uint8_t hwRevision = readMemoryByte();
    DEBUG_PRINT(F("Revision @ user[16][6] = "));
    DEBUG_PRINTLNF(hwRevision, HEX);
    DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    setMemoryBank(0, false, false);

    // check OTP bank valid
    DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    uint8_t otpValid = getOTPBankValid();
    DEBUG_PRINT(F("OTP bank is "));
    DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    DEBUG_PRINTLN(F("Reading gyro offset values..."));
    int8_t xgOffset = getXGyroOffset();
    int8_t ygOffset = getYGyroOffset();
    int8_t zgOffset = getZGyroOffset();
    DEBUG_PRINT(F("X gyro offset = "));
    DEBUG_PRINTLN(xgOffset);
    DEBUG_PRINT(F("Y gyro offset = "));
    DEBUG_PRINTLN(ygOffset);
    DEBUG_PRINT(F("Z gyro offset = "));
    DEBUG_PRINTLN(zgOffset);

    I2Cdev::readByte(devAddr, MPU6050_RA_USER_CTRL, buffer); // ?

    DEBUG_PRINTLN(F("Enabling interrupt latch, clear on any read, AUX bypass enabled"));
    I2Cdev::writeByte(devAddr, MPU6050_RA_INT_PIN_CFG, 0x32);

    // enable MPU AUX I2C bypass mode
    //DEBUG_PRINTLN(F("Enabling AUX I2C bypass mode..."));
    //setI2CBypassEnabled(true);

    DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    DEBUG_PRINTLN(F("Setting magnetometer mode to fuse access..."));
    //mag -> setMode(0x0F);
    I2Cdev::writeByte(0x0E, 0x0A, 0x0F);

    DEBUG_PRINTLN(F("Reading mag magnetometer factory calibration..."));
    int8_t asax, asay, asaz;
    //mag -> getAdjustment(&asax, &asay, &asaz);
    I2Cdev::readBytes(0x0E, 0x10, 3, buffer);
    asax = (int8_t)buffer[0];
    asay = (int8_t)buffer[1];
    asaz = (int8_t)buffer[2];
    DEBUG_PRINT(F("Adjustment X/Y/Z = "));
    DEBUG_PRINT(asax);
    DEBUG_PRINT(F(" / "));
    DEBUG_PRINT(asay);
    DEBUG_PRINT(F(" / "));
    DEBUG_PRINTLN(asaz);

    DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    // load DMP code into memory banks
    DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
    DEBUG_PRINTLN(F(" bytes)"));
    if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {
        DEBUG_PRINTLN(F("Success! DMP code written and verified."));

        DEBUG_PRINTLN(F("Configuring DMP and related settings..."));

        // write DMP configuration
        DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
        DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
        DEBUG_PRINTLN(F(" bytes in config def)"));
        if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
            DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

            DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            setIntEnabled(0x12);

            DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
            setRate(4); // 1khz / (1 + 4) = 200 Hz

            DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
            setDLPFMode(MPU6050_DLPF_BW_42);

            DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

            DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
            setOTPBankValid(false);

            DEBUG_PRINTLN(F("Setting X/Y/Z gyro offsets to previous values..."));
            setXGyroOffsetTC(xgOffset);
            setYGyroOffsetTC(ygOffset);
            setZGyroOffsetTC(zgOffset);

            //DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            //setXGyroOffset(0);
            //setYGyroOffset(0);
            //setZGyroOffset(0);

            DEBUG_PRINTLN(F("Writing final memory update 1/19 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 2/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            DEBUG_PRINTLN(F("Reading FIFO count..."));
            uint8_t fifoCount = getFIFOCount();

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);
            uint8_t fifoBuffer[128];
            //getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Writing final memory update 3/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 4/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Disabling all standby flags..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_PWR_MGMT_2, 0x00);

            DEBUG_PRINTLN(F("Setting accelerometer sensitivity to +/- 2g..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_ACCEL_CONFIG, 0x00);

            DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
            setMotionDetectionThreshold(2);

            DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
            setZeroMotionDetectionThreshold(156);

            DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
            setMotionDetectionDuration(80);

            DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
            setZeroMotionDetectionDuration(0);

            DEBUG_PRINTLN(F("Setting AK8975 to single measurement mode..."));
            //mag -> setMode(1);
            I2Cdev::writeByte(0x0E, 0x0A, 0x01);

            // setup AK8975 (0x0E) as Slave 0 in read mode
            DEBUG_PRINTLN(F("Setting up AK8975 read slave 0..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_ADDR, 0x8E);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_REG,  0x01);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV0_CTRL, 0xDA);

            // setup AK8975 (0x0E) as Slave 2 in write mode
            DEBUG_PRINTLN(F("Setting up AK8975 write slave 2..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_ADDR, 0x0E);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_REG,  0x0A);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_CTRL, 0x81);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV2_DO,   0x01);

            // setup I2C timing/delay control
            DEBUG_PRINTLN(F("Setting up slave access delay..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_SLV4_CTRL, 0x18);
            I2Cdev::writeByte(0x68, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x05);

            // enable interrupts
            DEBUG_PRINTLN(F("Enabling default interrupt behavior/no bypass..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_INT_PIN_CFG, 0x00);

            // enable I2C master mode and reset DMP/FIFO
            DEBUG_PRINTLN(F("Enabling I2C master mode..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_USER_CTRL, 0x20);
            DEBUG_PRINTLN(F("Resetting FIFO..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_USER_CTRL, 0x24);
            DEBUG_PRINTLN(F("Rewriting I2C master mode enabled because...I don't know"));
            I2Cdev::writeByte(0x68, MPU6050_RA_USER_CTRL, 0x20);
            DEBUG_PRINTLN(F("Enabling and resetting DMP/FIFO..."));
            I2Cdev::writeByte(0x68, MPU6050_RA_USER_CTRL, 0xE8);

            DEBUG_PRINTLN(F("Writing final memory update 5/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 6/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 7/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 8/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 9/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 10/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 11/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Reading final memory update 12/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            #ifdef DEBUG
                DEBUG_PRINT(F("Read bytes: "));
                for (j = 0; j < 4; j++) {
                    DEBUG_PRINTF(dmpUpdate[3 + j], HEX);
                    DEBUG_PRINT(" ");
                }
                DEBUG_PRINTLN("");
            #endif

            DEBUG_PRINTLN(F("Writing final memory update 13/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 14/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 15/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 16/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            DEBUG_PRINTLN(F("Writing final memory update 17/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIRO count >= 46..."));
            while ((fifoCount = getFIFOCount()) < 46);
            DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();

            DEBUG_PRINTLN(F("Writing final memory update 18/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIRO count >= 48..."));
            while ((fifoCount = getFIFOCount()) < 48);
            DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();
            DEBUG_PRINTLN(F("Waiting for FIRO count >= 48..."));
            while ((fifoCount = getFIFOCount()) < 48);
            DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, min(fifoCount, 128)); // safeguard only 128 bytes
            DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();

            DEBUG_PRINTLN(F("Writing final memory update 19/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
            setDMPEnabled(false);

            DEBUG_PRINTLN(F("Setting up internal 48-byte (default) DMP packet buffer..."));
            dmpPacketSize = 48;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            resetFIFO();
            getIntStatus();
        } else {
            DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
            return 2; // configuration block loading failed
        }
    } else {
        DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        return 1; // main binary block loading failed
    }
    return 0; // success
}

bool MPU6050_9::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU6050_9::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[34] << 24) | ((uint32_t)packet[35] << 16) | ((uint32_t)packet[36] << 8) | packet[37]);
    data[1] = (((uint32_t)packet[38] << 24) | ((uint32_t)packet[39] << 16) | ((uint32_t)packet[40] << 8) | packet[41]);
    data[2] = (((uint32_t)packet[42] << 24) | ((uint32_t)packet[43] << 16) | ((uint32_t)packet[44] << 8) | packet[45]);
    return 0;
}

uint8_t MPU6050_9::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[34] << 8) | packet[35];
    data[1] = (packet[38] << 8) | packet[39];
    data[2] = (packet[42] << 8) | packet[43];
    return 0;
}

uint8_t MPU6050_9::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[34] << 8) | packet[35];
    v -> y = (packet[38] << 8) | packet[39];
    v -> z = (packet[42] << 8) | packet[43];
    return 0;
}

uint8_t MPU6050_9::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}

uint8_t MPU6050_9::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}

uint8_t MPU6050_9::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t MPU6050_9::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 24) | ((uint32_t)packet[17] << 16) | ((uint32_t)packet[18] << 8) | packet[19]);
    data[1] = (((uint32_t)packet[20] << 24) | ((uint32_t)packet[21] << 16) | ((uint32_t)packet[22] << 8) | packet[23]);
    data[2] = (((uint32_t)packet[24] << 24) | ((uint32_t)packet[25] << 16) | ((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}

uint8_t MPU6050_9::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[20] << 8) | packet[21];
    data[2] = (packet[24] << 8) | packet[25];
    return 0;
}

uint8_t MPU6050_9::dmpGetMag(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) | packet[29];
    data[1] = (packet[30] << 8) | packet[31];
    data[2] = (packet[32] << 8) | packet[33];
    return 0;
}

uint8_t MPU6050_9::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
    return 0;
}

uint8_t MPU6050_9::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}

uint8_t MPU6050_9::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t MPU6050_9::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

uint8_t MPU6050_9::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

uint8_t MPU6050_9::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}

uint8_t MPU6050_9::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;

        // increment external process count variable, if supplied
        if (processed != 0) *processed++;
    }
    return 0;
}

uint16_t MPU6050_9::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}

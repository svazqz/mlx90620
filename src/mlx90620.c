#define _POSIX_C_SOURCE 200809L
#include <mlx90620.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

enum { SENSOR_ADDRESS = 0x60, EEPROM_ADDRESS = 0x50 };

static uint16_t unsigned_word(const uint8_t data[2])
{
    return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int16_t signed_word(uint16_t value)
{
    return (int16_t)(value < 0x8000 ? (int32_t)value : (int32_t)value - 65536);
}

static int signed_byte(uint8_t value)
{
    return value < 0x80 ? value : value - 256;
}

static void settle(void)
{
    struct timespec remaining = { 0, 5000000L };
    while (nanosleep(&remaining, &remaining) == -1 && errno == EINTR) {
    }
}

static bool transfer(uint8_t address, const uint8_t *command, size_t command_size,
                     uint8_t *data, size_t size)
{
    i2c_t bus = *getBus();
    i2c_start(bus);
    bool ok = i2c_send_byte(bus, (address << 1) | I2C_WRITE) == I2C_ACK;
    for (size_t index = 0; ok && index < command_size; ++index)
        ok = i2c_send_byte(bus, command[index]) == I2C_ACK;

    if (ok && size != 0) {
        i2c_start(bus);
        ok = i2c_send_byte(bus, (address << 1) | I2C_READ) == I2C_ACK;
        for (size_t index = 0; ok && index < size; ++index) {
            data[index] = i2c_read_byte(bus);
            i2c_send_bit(bus, index + 1 == size ? I2C_NACK : I2C_ACK);
            ok = i2c_status() == I2C_OK;
        }
    }
    i2c_stop(bus);
    return ok && i2c_status() == I2C_OK;
}

static bool read_eeprom(uint8_t eeprom[256])
{
    const uint8_t start_address = 0;
    return transfer(EEPROM_ADDRESS, &start_address, 1, eeprom, 256);
}

static bool write_trim(uint8_t trim)
{
    const uint8_t command[] = { 0x04, (uint8_t)(trim - 0xaa), trim, 0x56, 0x00 };
    return transfer(SENSOR_ADDRESS, command, sizeof(command), NULL, 0);
}

static bool configure(void)
{
    const uint8_t config_high = 0x44;
    const uint8_t command[] = {
        0x03, (uint8_t)(Hz_LSB_32 - 0x55), Hz_LSB_32,
        (uint8_t)(config_high - 0x55), config_high
    };
    return transfer(SENSOR_ADDRESS, command, sizeof(command), NULL, 0);
}

static bool read_register(uint8_t address, uint16_t *value)
{
    const uint8_t command[] = { 0x02, address, 0x00, 0x01 };
    uint8_t bytes[2];
    if (!transfer(SENSOR_ADDRESS, command, sizeof(command), bytes, sizeof(bytes)))
        return false;
    *value = unsigned_word(bytes);
    return true;
}

void initSensor(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;

    uint8_t eeprom[256];
    uint16_t configuration;
    sensor->check = MLX90620_ERROR;
    if (!read_eeprom(eeprom) || !write_trim(eeprom[OSC_TRIM_VALUE]))
        return;
    settle();
    if (!configure())
        return;
    settle();
    if (!readConfigChecked(&configuration))
        return;

    memcpy(sensor->eeprom, eeprom, sizeof(eeprom));
    sensor->resolution = (configuration & 0x30) >> 4;
    sensor->check = MLX90620_OK;
}

void readEEPROM(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;
    uint8_t eeprom[256];
    sensor->check = read_eeprom(eeprom) ? MLX90620_OK : MLX90620_ERROR;
    if (sensor->check == MLX90620_OK)
        memcpy(sensor->eeprom, eeprom, sizeof(eeprom));
}

void writeTrimmingValue(MLX90620 *sensor)
{
    if (sensor != NULL)
        sensor->check = write_trim(sensor->eeprom[OSC_TRIM_VALUE]) ? MLX90620_OK : MLX90620_ERROR;
}

void setConfiguration(void)
{
    (void)configure();
}

bool readConfigChecked(uint16_t *config)
{
    return config != NULL && read_register(0x92, config);
}

int16_t readConfig(void)
{
    uint16_t config = 0;
    (void)readConfigChecked(&config);
    return signed_word(config);
}

bool checkConfig(void)
{
    uint16_t config;
    return !readConfigChecked(&config) || (config & 0x0400) == 0;
}

void readRes(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;
    uint16_t config;
    sensor->check = readConfigChecked(&config) ? MLX90620_OK : MLX90620_ERROR;
    if (sensor->check == MLX90620_OK)
        sensor->resolution = (config & 0x30) >> 4;
}

void readPTAT(MLX90620 *sensor)
{
    if (sensor != NULL)
        sensor->check = read_register(0x40, &sensor->ptat) ? MLX90620_OK : MLX90620_ERROR;
}

void readCPIX(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;
    uint16_t value;
    sensor->check = read_register(0x41, &value) ? MLX90620_OK : MLX90620_ERROR;
    if (sensor->check == MLX90620_OK)
        sensor->cpix = signed_word(value);
}

void readIR(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;
    const uint8_t command[] = { 0x02, 0x00, 0x01, 0x40 };
    uint8_t pixels[128];
    sensor->check = transfer(SENSOR_ADDRESS, command, sizeof(command), pixels, sizeof(pixels))
        ? MLX90620_OK : MLX90620_ERROR;
    if (sensor->check == MLX90620_OK) {
        for (size_t index = 0; index < 64; ++index)
            sensor->irData[index] = signed_word(unsigned_word(pixels + 2 * index));
    }
}

void calculateTA(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;

    sensor->check = MLX90620_ERROR;
    sensor->tambient = NAN;
    if (sensor->resolution < 0 || sensor->resolution > 3)
        return;

    const uint8_t *cal = sensor->eeprom;
    int scale = sensor->resolution - 3;
    double threshold = ldexp(signed_word(unsigned_word(cal + VTH_L)), scale);
    double linear = ldexp(signed_word(unsigned_word(cal + KT1_L)), scale - (cal[KT_SCALE] >> 4));
    double quadratic = ldexp(signed_word(unsigned_word(cal + KT2_L)),
                             scale - (cal[KT_SCALE] & 0x0f) - 10);
    double delta = sensor->ptat - threshold;
    double discriminant = linear * linear + 4 * quadratic * delta;
    if (discriminant < 0.0 || (quadratic == 0.0 && linear == 0.0))
        return;

    double temperature = quadratic == 0.0
        ? 25.0 + delta / linear
        : 25.0 + (sqrt(discriminant) - linear) / (2 * quadratic);
    if (!isfinite(temperature) || fabs(temperature) > FLT_MAX)
        return;

    sensor->tambient = (float)temperature;
    sensor->check = MLX90620_OK;
}

void calculateTO(MLX90620 *sensor)
{
    if (sensor == NULL)
        return;

    for (size_t index = 0; index < 64; ++index)
        sensor->temperatures[index] = NAN;
    sensor->check = MLX90620_ERROR;
    if (sensor->resolution < 0 || sensor->resolution > 3 ||
        !isfinite(sensor->tambient) || sensor->tambient < -273.15f)
        return;

    const uint8_t *cal = sensor->eeprom;
    double emissivity = ldexp(unsigned_word(cal + CAL_EMIS_L), -15);
    if (emissivity <= 0.0 || emissivity > 1.0)
        return;

    int scale = sensor->resolution - 3;
    int a_scale = cal[CAL_AI_SCALE] >> 4;
    int b_scale = scale - (cal[CAL_BI_SCALE] & 0x0f);
    double delta = sensor->tambient - 25.0;
    double a_common = signed_word(unsigned_word(cal + CAL_ACOMMON_L));
    double a_cp = ldexp(signed_word(unsigned_word(cal + CAL_ACP_L)), scale);
    double b_cp = ldexp(signed_byte(cal[CAL_BCP]), b_scale);
    double tgc = signed_byte(cal[CAL_TGC]) / 32.0;
    double alpha_cp = ldexp(unsigned_word(cal + CAL_alphaCP_L), scale - cal[CAL_A0_SCALE]);
    double common_sensitivity = ldexp(unsigned_word(cal + CAL_A0_L), -cal[CAL_A0_SCALE]);
    double ambient_radiance = pow(sensor->tambient + 273.15, 4);
    double offset_cp = sensor->cpix - (a_cp + b_cp * delta);
    sensor->check = MLX90620_OK;

    for (size_t index = 0; index < 64; ++index) {
        double a = ldexp(a_common + ldexp(cal[index], a_scale), scale);
        double b = ldexp(signed_byte(cal[0x40 + index]), b_scale);
        double sensitivity = ldexp(common_sensitivity +
                                   ldexp(cal[0x80 + index], -cal[CAL_DELTA_A_SCALE]), scale);
        double gain = (sensitivity - tgc * alpha_cp) * emissivity;
        if (gain <= 0.0) {
            sensor->check = MLX90620_ERROR;
            continue;
        }
        double signal = sensor->irData[index] - (a + b * delta) - tgc * offset_cp;
        double radiance = signal / gain + ambient_radiance;
        if (!isfinite(radiance) || radiance < 0.0) {
            sensor->check = MLX90620_ERROR;
            continue;
        }
        sensor->temperatures[index] = (float)(sqrt(sqrt(radiance)) - 273.15);
    }
}

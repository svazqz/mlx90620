#include "mock_i2c.h"
#include <mlx90620.h>
#include <assert.h>
#include <math.h>
#include <string.h>

static void expect_writes(const uint8_t *writes, size_t size)
{
    assert(bus_mock.write_count == size);
    assert(memcmp(bus_mock.writes, writes, size) == 0);
}

static void expect_read_end(size_t size)
{
    assert(bus_mock.read_count == size && bus_mock.reply_count == size);
    for (size_t index = 0; index + 1 < size; ++index)
        assert(bus_mock.replies[index] == I2C_ACK);
    assert(bus_mock.replies[size - 1] == I2C_NACK);
}

static void initialization_and_eeprom(void)
{
    uint8_t bytes[258];
    for (size_t index = 0; index < 256; ++index)
        bytes[index] = (uint8_t)index;
    bytes[256] = 0x20;
    bytes[257] = 0x44;
    MLX90620 sensor = { 0 };

    mock_load(bytes, sizeof(bytes));
    initSensor(&sensor);
    assert(sensor.check == MLX90620_OK && sensor.resolution == 2);
    assert(memcmp(sensor.eeprom, bytes, 256) == 0);
    const uint8_t init_writes[] = {
        0xa0, 0x00, 0xa1, 0xc0, 0x04, 0x4d, 0xf7, 0x56, 0x00,
        0xc0, 0x03, 0xe4, 0x39, 0xef, 0x44, 0xc0, 0x02, 0x92, 0x00, 0x01, 0xc1
    };
    expect_writes(init_writes, sizeof(init_writes));
    assert(bus_mock.starts == 6 && bus_mock.stops == 4);
    assert(bus_mock.replies[255] == I2C_NACK && bus_mock.replies[257] == I2C_NACK);

    mock_load(bytes, 256);
    readEEPROM(&sensor);
    assert(sensor.check == MLX90620_OK);
    expect_read_end(256);

    mock_load(NULL, 0);
    writeTrimmingValue(&sensor);
    assert(sensor.check == MLX90620_OK);
    expect_writes(init_writes + 3, 6);

    mock_load(NULL, 0);
    setConfiguration();
    expect_writes(init_writes + 9, 6);

    mock_load(bytes, sizeof(bytes));
    bus_mock.fail_at_read = 13;
    memset(sensor.eeprom, 0xa5, sizeof(sensor.eeprom));
    initSensor(&sensor);
    assert(sensor.check == MLX90620_ERROR && sensor.resolution == 2);
    assert(sensor.eeprom[0] == 0xa5 && sensor.eeprom[255] == 0xa5);
    assert(bus_mock.stops == 1);
}

static void registers_and_pixels(void)
{
    MLX90620 sensor = { 0 };
    const uint8_t high_word[] = { 0x34, 0xf2 };
    const uint8_t config_word[] = { 0x30, 0x44 };
    const uint8_t config_command[] = { 0xc0, 0x02, 0x92, 0, 1, 0xc1 };
    uint16_t config = 0;
    mock_load(config_word, sizeof(config_word));
    assert(readConfigChecked(&config) && config == 0x4430);
    expect_writes(config_command, sizeof(config_command));
    expect_read_end(2);
    mock_load(config_word, sizeof(config_word));
    assert(readConfig() == 0x4430);
    mock_load(config_word, sizeof(config_word));
    assert(!checkConfig());
    mock_load(high_word, sizeof(high_word));
    assert(checkConfig());
    mock_load(config_word, sizeof(config_word));
    readRes(&sensor);
    assert(sensor.resolution == 3 && sensor.check == MLX90620_OK);

    mock_load(high_word, sizeof(high_word));
    readPTAT(&sensor);
    assert(sensor.ptat == 0xf234 && bus_mock.writes[2] == 0x40);
    mock_load(high_word, sizeof(high_word));
    readCPIX(&sensor);
    assert(sensor.cpix == -3532 && bus_mock.writes[2] == 0x41);

    uint8_t pixels[128] = { 0x00, 0x80 };
    pixels[126] = 0xff;
    pixels[127] = 0x7f;
    mock_load(pixels, sizeof(pixels));
    readIR(&sensor);
    assert(sensor.check == MLX90620_OK && sensor.irData[0] == -32768 && sensor.irData[63] == 32767);
    const uint8_t image_command[] = { 0xc0, 0x02, 0, 1, 0x40, 0xc1 };
    expect_writes(image_command, sizeof(image_command));
    expect_read_end(sizeof(pixels));

    mock_load(NULL, 0);
    bus_mock.nack_at_write = 1;
    readPTAT(&sensor);
    assert(sensor.check == MLX90620_ERROR && sensor.ptat == 0xf234);
    mock_load(NULL, 0);
    bus_mock.nack_at_write = 1;
    assert(checkConfig() && bus_mock.stops == 1);
    mock_load(NULL, 0);
    bus_mock.nack_at_write = 1;
    assert(!readConfigChecked(&config) && config == 0x4430);
    assert(!readConfigChecked(NULL));
}

static void temperatures(void)
{
    MLX90620 sensor = { .resolution = 3, .ptat = 110 };
    sensor.eeprom[VTH_L] = 100;
    sensor.eeprom[KT1_L] = 2;
    calculateTA(&sensor);
    assert(sensor.check == MLX90620_OK && sensor.tambient == 30.0f);

    sensor.eeprom[KT2_H] = 4;
    sensor.ptat = 103;
    calculateTA(&sensor);
    assert(sensor.check == MLX90620_OK && sensor.tambient == 26.0f);
    sensor.ptat = 0;
    calculateTA(&sensor);
    assert(sensor.check == MLX90620_ERROR && isnan(sensor.tambient));
    sensor.eeprom[KT1_L] = sensor.eeprom[KT2_H] = 0;
    calculateTA(&sensor);
    assert(sensor.check == MLX90620_ERROR);

    sensor.tambient = 25.0f;
    sensor.eeprom[CAL_EMIS_H] = 0x80;
    sensor.eeprom[CAL_A0_L] = 1;
    calculateTO(&sensor);
    assert(sensor.check == MLX90620_OK);
    for (size_t index = 0; index < 64; ++index)
        assert(fabsf(sensor.temperatures[index] - 25.0f) < 0.001f);

    sensor.eeprom[CAL_A0_SCALE] = 28;
    sensor.irData[0] = -500;
    calculateTO(&sensor);
    assert(sensor.check == MLX90620_ERROR && isnan(sensor.temperatures[0]));
    assert(fabsf(sensor.temperatures[63] - 25.0f) < 0.001f);
    sensor.eeprom[CAL_A0_L] = 0;
    calculateTO(&sensor);
    assert(sensor.check == MLX90620_ERROR && isnan(sensor.temperatures[63]));

    sensor.eeprom[CAL_EMIS_H] = 0;
    calculateTO(&sensor);
    assert(sensor.check == MLX90620_ERROR && isnan(sensor.temperatures[0]));
    sensor.resolution = -1;
    calculateTA(&sensor);
    calculateTO(&sensor);
    assert(sensor.check == MLX90620_ERROR && isnan(sensor.tambient));
}

int main(void)
{
    initialization_and_eeprom();
    registers_and_pixels();
    temperatures();
    return 0;
}

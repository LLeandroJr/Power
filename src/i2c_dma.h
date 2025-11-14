#include "driver/i2c.h"
#include "clib/u8g2.h"

// ======== CONFIGURAÇÕES DE PINOS ========
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_PORT I2C_NUM_0
#define I2C_MASTER_SDA_IO   4
#define I2C_MASTER_SCL_IO   15
#define I2C_MASTER_FREQ_HZ  100000

// ======== ENDEREÇO DO DISPLAY ========
// SSD1306 padrão (0x3C)
#define OLED_ADDR           0x3C

// ======== CALLBACK I2C COM DMA ========
uint8_t u8x8_byte_i2c_dma(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[256]; // buffer temporário (DMA friendly)
    static uint8_t buf_pos = 0;

    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            // Nada a inicializar aqui, já configuramos o driver abaixo
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_pos = 0;
            break;

        case U8X8_MSG_BYTE_SEND: {
            uint8_t *data = (uint8_t *)arg_ptr;
            for (uint8_t i = 0; i < arg_int; i++) {
                buffer[buf_pos++] = data[i];
            }
            break;
        }

        case U8X8_MSG_BYTE_END_TRANSFER:
            i2c_master_write_to_device(I2C_MASTER_NUM,
                                       u8x8_GetI2CAddress(u8x8) >> 1,
                                       buffer, buf_pos,
                                       pdMS_TO_TICKS(100));
            break;

        default:
            return 0;
    }
    return 1;
}

// ======== CALLBACK GPIO e DELAY ========
uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
        default:
            break;
    }
    return 1;
}

// ======== INICIALIZAÇÃO DO I2C COM DMA ========
void i2c_master_init(void) {
    i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        conf.clk_flags = 0;

    i2c_param_config(I2C_MASTER_NUM, &conf);

    // Instala driver (modo master)
    // DMA é ativado automaticamente em transferências >32 bytes
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_write_dma(uint8_t addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    // 🧠 É aqui que o driver escolhe DMA automaticamente (para len > 32)
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
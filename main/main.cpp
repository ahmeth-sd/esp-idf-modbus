#include "Arduino.h"          // Arduino bileşenini dahil et
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TXD1 48
#define RXD1 47
#define UART_PORT UART_NUM_1
#define BUF_SIZE 256

static const char *TAG = "MODBUS";

// ======== CRC HESABI ========
uint16_t ModRTU_CRC(const uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ======== UART'a frame gönder ========
void send_frame(const uint8_t* data, int len) {
    uart_write_bytes(UART_PORT, (const char*)data, len);
}

// ======== Modbus WRITE SINGLE REGISTER ========
void write_single_register(uint8_t slave_id, uint16_t address, uint16_t value) {
    uint8_t frame[8];

    frame[0] = slave_id;
    frame[1] = 0x06;
    frame[2] = (address >> 8) & 0xFF;
    frame[3] = address & 0xFF;
    frame[4] = (value >> 8) & 0xFF;
    frame[5] = value & 0xFF;

    uint16_t crc = ModRTU_CRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    ESP_LOGI(TAG, "Write Frame:");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");

    send_frame(frame, 8);
}

void open_motor(int frequency) {
    write_single_register(1, 100, frequency);
    delay(100);  // Arduino delay fonksiyonu
    write_single_register(1, 99, 1151);
}

void close_motor(int frequency) {
    write_single_register(1, 100, frequency);
    delay(100);
    write_single_register(1, 99, 3199);
}

void stop_motor() {
    write_single_register(1, 100, 0);
    delay(100);
    write_single_register(1, 99, 1278);
    delay(100);
    write_single_register(1, 99, 1150);
}

// ======== Modbus READ HOLDING REGISTERS ========
void modbus_read_holding_registers(uint8_t slave_id, uint16_t start_addr, uint16_t num_regs) {
    uint8_t frame[8];

    frame[0] = slave_id;
    frame[1] = 0x03;
    frame[2] = (start_addr >> 8) & 0xFF;
    frame[3] = start_addr & 0xFF;
    frame[4] = (num_regs >> 8) & 0xFF;
    frame[5] = num_regs & 0xFF;

    uint16_t crc = ModRTU_CRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    send_frame(frame, 8);
}

bool read_modbus_response(uint8_t expected_slave_id, uint8_t* buffer, int max_len, uint8_t quantity) {
    int expected_len = 3 + quantity * 2 + 2;
    int len = uart_read_bytes(UART_PORT, buffer, expected_len, pdMS_TO_TICKS(100));

    if (len < expected_len) {
        ESP_LOGW(TAG, "Eksik cevap! %d/%d byte", len, expected_len);
        return false;
    }

    if (buffer[0] == expected_slave_id && buffer[1] == 0x03) {
        uint16_t crc_received = buffer[expected_len-2] | (buffer[expected_len-1] << 8);
        uint16_t crc_calc = ModRTU_CRC(buffer, expected_len-2);

        if (crc_received == crc_calc) {
            uint8_t byte_count = buffer[2];
            for (int i = 0; i < byte_count / 2; i++) {
                uint16_t reg = (buffer[3 + i*2] << 8) | buffer[4 + i*2];
                ESP_LOGI(TAG, "Reg[%d] = %u", i, reg);
            }
            return true;
        } else {
            ESP_LOGE(TAG, "CRC hatasi! Hesap:%04X Gelen:%04X", crc_calc, crc_received);
        }
    }
    return false;
}

// ======== Arduino as a component entegrasyonu ========
extern "C" void app_main() {
    // Arduino ortamını başlat
    initArduino();

    // UART1 yapılandırması
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD1, RXD1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    Serial.begin(115200);
    Serial.println("🔧 Modbus Master başlatıldı (ESP-IDF + Arduino)");

    uint8_t response[256];
    uint8_t quantity = 2;

    while (true) {
        modbus_read_holding_registers(2, 0x0000, quantity);
        delay(30);

        if (read_modbus_response(2, response, sizeof(response), quantity)) {
            uint16_t command = (response[5] << 8) | response[6];
            switch (command) {
                case 1:
                    Serial.println(">> Komut: OPEN");
                    open_motor(5000);
                    delay(50);
                    while (Serial1.available()) Serial1.read();  // RX buffer temizle
                    write_single_register(2, 1, 0);
                    delay(50);
                    while (Serial1.available()) Serial1.read();  // RX buffer temizle
                    continue;;
                case 2:
                    Serial.println(">> Komut: CLOSE");
                    close_motor(50);
                    delay(50);
                    write_single_register(2, 1, 0);
                    continue;;
                case 3:
                    Serial.println(">> Komut: STOP");
                    stop_motor();
                    delay(50);
                    write_single_register(2, 1, 0);
                    continue;;
                default:
                    continue;;
            }
        } else {
            Serial.println("Okuma hatası.");
        }

        delay(300);
    }
}

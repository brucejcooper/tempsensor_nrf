#ifndef TMP11X_H
#define TMP11X_H

#ifdef __cplusplus
extern "C" {
#endif


#define TMP11X_ADDRESS                  0x48

#define TMP11X_CONFIG_HIGH_ALERT        (1 << 15)
#define TMP11X_CONFIG_LOW_ALERT         (1 << 14)
#define TMP11X_CONFIG_DATA_READY        (1 << 13)
#define TMP11X_CONFIG_EEPROM_BUSY       (1 << 12)
#define TMP11X_CONFIG_MODE_CONTINUOUS   (0b00 << 10)
#define TMP11X_CONFIG_MODE_SHUTDOWN     (0b01 << 10)
#define TMP11X_CONFIG_MODE_ONESHOT      (0b11 << 10)
#define TMP11X_CONFIG_CONV_15ms         (0b000 << 7)
#define TMP11X_CONFIG_CONV_125ms        (0b001 << 7)
#define TMP11X_CONFIG_CONV_250ms        (0b010 << 7)
#define TMP11X_CONFIG_CONV_500ms        (0b011 << 7)
#define TMP11X_CONFIG_CONV_1s           (0b100 << 7)
#define TMP11X_CONFIG_CONV_4s           (0b101 << 7)
#define TMP11X_CONFIG_CONV_8s           (0b110 << 7)
#define TMP11X_CONFIG_CONV_16s          (0b111 << 7)
#define TMP11X_CONFIG_AVG1              (0b00 << 5)
#define TMP11X_CONFIG_AVG8              (0b01 << 5)
#define TMP11X_CONFIG_AVG32             (0b10 << 5)
#define TMP11X_CONFIG_AVG64             (0b11 << 5)
#define TMP11X_CONFIG_THERM_MODE        (1 << 4)
#define TMP11X_CONFIG_ALERT_ACTIVE_HIGH (1 << 3)
#define TMP11X_CONFIG_ALERT_ACTIVE_LOW  (0 << 3)
#define TMP11X_CONFIG_ALERT_SHOWS_DRDY  (1 << 2)
#define TMP11X_CONFIG_ALERT_SHOWS_ALERT (0 << 2)

typedef void (*tmp11x_callback_t)(float temperature);

void tmp11x_init();
void tmp11x_initiate_read();

#ifdef __cplusplus
}
#endif

#endif
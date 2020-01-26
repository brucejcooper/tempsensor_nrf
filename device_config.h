#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#define CCPEED_CFG_DEVICE_NAME_MAXLEN 32
#define CCPEED_CFG_DEVICE_NAME_MAXLEN_INTS (CCPEED_CFG_DEVICE_NAME_MAXLEN/4)



typedef struct {
    char device_name[CCPEED_CFG_DEVICE_NAME_MAXLEN];
    uint8_t sampling_period;
} ccpeed_cfg_t;

extern ccpeed_cfg_t ccpeed_cfg;


#ifdef __cplusplus
}
#endif

#endif /* NRF51_H */


/*
 * imxrt_pico_eth_dev.h
 *
 *  Created on: Dec 19, 2017
 *      Author: venturus
 */
#ifndef __IMXRT_ENET_H_
#define __IMXRT_ENET_H_

/**
 *  @fn pico_eth_create()
 *  @brief creates a ethernet device based on IMXRT enet hw
 *
 */
struct pico_device *pico_eth_create(const char *name, const uint8_t *mac);


#endif

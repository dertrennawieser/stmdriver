/*
 * rtc.h
 *
 *  Created on: 28.07.2022
 *      Author: moritz
 */

#ifndef RTC_H_
#define RTC_H_

#include "stm32l0xx.h"

void rtc_init();
void rtc_periodicwakeup(uint16_t sec);

#endif /* RTC_H_ */

/*

  WS2812B CPU and memory efficient library

  Date: 28.9.2016

  Author: Martin Hubacek
  	  	  http://www.martinhubacek.cz
  	  	  @hubmartin

  Licence: MIT License

*/

#ifndef VISEFFECT_H_
#define VISEFFECT_H_

#include <stdint.h>
#include "usb_device.h"
#include "usbd_customhid.h"
#include "main.h"

typedef struct {
	uint8_t buttons;
	uint16_t axis_x;
	uint16_t axis_y;
} USBD_JoystickReport_TypeDef;

extern USBD_JoystickReport_TypeDef joystickReport;

void visInit(void);
void visHandle(void);
void visHandle2(void);
void visScript(uint8_t *frameBuffer, uint32_t frameBufferSize);

#endif /* VISEFFECT_H_ */

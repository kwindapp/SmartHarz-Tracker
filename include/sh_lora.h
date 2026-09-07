#ifndef _SH_LORA_H_
#define _SH_LORA_H_

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#include "t-echo.h"

bool loraInit();
bool loraSendTest();
void loraLoop();

#endif
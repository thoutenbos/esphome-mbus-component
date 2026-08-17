#pragma once

#include "../mbus.h"

namespace esphome {
namespace mbus {
	
	//variable-length Data Record-specific definitions
static const uint8_t MBUS_DIF_DATATYPE_MASK = 0x0F;
static const uint8_t MBUS_DIF_FUNCTION_MASK = 0x30;
static const uint8_t MBUS_DIF_STORAGE_MASK = 0x40;
static const uint8_t MBUS_DIF_EXTENSION_MASK = 0x80;

static const uint8_t MBUS_DIFE_MAX = 10;
static const uint8_t MBUS_DIFE_EXTENSION_MASK = 0X80;
static const uint8_t MBUS_DIFE_SUBUNIT_MASK = 0X40;
static const uint8_t MBUS_DIFE_TARIFF_MASK = 0X30;
static const uint8_t MBUS_DIFE_STORAGE_MASK = 0X0F;

static const uint8_t MBUS_VIFE_MAX = 11; //VIF + 10 VIFEs
static const uint8_t MBUS_VIFE_EXTENSION_MASK = 0X80;


}  // namespace mbus
}  // namespace esphome

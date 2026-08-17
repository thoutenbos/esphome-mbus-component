#include "mbus_datarecord.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mbus {
	
static const char *const TAG = "mbus.datarecord";

const char* MbusDIFDatatypeToStr(enum MbusDIFDatatype datatype){
  switch (datatype){
  case MBUS_NO_DATA: return "MBUS_NO_DATA"; break;
  case MBUS_INT_8BIT: return "MBUS_INT_8BIT"; break;
  case MBUS_INT_16BIT: return "MBUS_INT_16BIT"; break;
  case MBUS_INT_24BIT: return "MBUS_INT_24BIT"; break;
  case MBUS_INT_32BIT: return "MBUS_INT_32BIT"; break;
  case MBUS_INT_48BIT: return "MBUS_INT_48BIT"; break;
  case MBUS_INT_64BIT: return "MBUS_INT_64BIT"; break;
  case MBUS_REAL: return "MBUS_REAL"; break;
  case MBUS_SELECTION: return "MBUS_SELECTION"; break;
  case MBUS_BCD2: return "MBUS_BCD2"; break;
  case MBUS_BCD4: return "MBUS_BCD4"; break;
  case MBUS_BCD6: return "MBUS_BCD6"; break;
  case MBUS_BCD8: return "MBUS_BCD8"; break;
  case MBUS_VARIABLE_LEN: return "MBUS_VARIABLE_LEN"; break;
  case MBUS_BCD12: return "MBUS_BCD12"; break;
  case MBUS_SPECIAL: return "MBUS_SPECIAL"; break;
  default: return "Unknown"; break;
  }
}

const char* MbusDIFFunctionToStr(enum MbusDIFFunction function){
  switch (function){
  case MBUS_INSTANT_VALUE: return "MBUS_INSTANT_VALUE"; break;
  case MBUS_MAXIMUM_VALUE: return "MBUS_MAXIMUM_VALUE"; break;
  case MBUS_MINIMUM_VALUE: return "MBUS_MINIMUM_VALUE"; break;
  case MBUS_ERROR_VALUE: return "MBUS_ERROR_VALUE"; break;
  default: return "Unknown"; break;
 }
}

/* void Mbus::parse_telegram():
 *
 * Parses the fixed header and every variable-length Data Record of the
 * just-received telegram (this->telegram) into this->records. Runs once per
 * new telegram - this used to be duplicated in every MbusSensor::loop().
 * */

void Mbus::parse_telegram() {

  this->records.clear();

  uint8_t* tg = this->telegram;
  uint16_t len = tg[1] + 6; //payload + (start + length + length + start + checksum + stop)

  //tg[0] to tg[2] checked by statemachine
  //tg[3] start byte

  if(tg[4] != MBUS_CONTROL_RSP_UD){
	  ESP_LOGE(TAG, "%llx: Unexpected control field %d", this->secondary_address, tg[4]);
	  return;
  }

  //tg[5] primary address (usually 0 anyway)

  if(tg[6] != MBUS_CI_RESP_VARIABLE){
	  ESP_LOGE(TAG, "%llx: Unexpected control information field %d", this->secondary_address, tg[6]);
	  return;
  }

  //tg[7] to tg[14] secondary address

  std::string secondary_address_str;
  char buf[5];
  uint16_t pos;
  for(pos=10;pos>=7;pos--){
	sprintf(buf, "%02X", tg[pos]);
    secondary_address_str += buf;
  }
  for(pos=11;pos<=14;pos++){
	sprintf(buf, "%02X", tg[pos]);
    secondary_address_str += buf;
  }
  ESP_LOGD(TAG, "%llx: Secondary address received: %s", this->secondary_address, secondary_address_str.c_str());

  //tg[15] access number

  if(tg[16] & MBUS_STATUS_APP_ERROR){
	  ESP_LOGE(TAG, "%llx: Application error", this->secondary_address);
	  return;
  }
  if(tg[16] & MBUS_STATUS_TEMPORARY_ERROR){
	   ESP_LOGW(TAG, "%llx: Temporary error status bit set", this->secondary_address);
  }
  if(tg[16] & MBUS_STATUS_PERMANENT_ERROR){
	  ESP_LOGW(TAG, "%llx: Permanent error status bit set. Consider replacing meter.", this->secondary_address);
  }
  if(tg[16] & MBUS_STATUS_LOW_POWER){
	  ESP_LOGW(TAG, "%llx: Low power status bit set. Consider replacing meter.", this->secondary_address);
  }

  //tg[17] to tg[18] signature
  if(tg[17] || tg[18]){
	  ESP_LOGE(TAG, "%llx: Nonzero signature field, possible encryption", this->secondary_address);
	  return;
  }

  ESP_LOGD(TAG, "%llx: Parsing fixed header done", this->secondary_address);

  //variable payload fields follow
  pos = 19;
  while( ( pos <= (len-3) ) &&
	( tg[pos] != MBUS_DIF_MANUFACTURER_SPECIFIC ) &&
	( tg[pos] != MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME ) )
  { //each iteration of the while loop processes one Data Record,
	  //beginning with pos pointing at DIF. Iteration ends with
	  //pos pointing at DIF of next Data Record.

	  if(tg[pos] == MBUS_DIF_FILLER) {
		  pos++;
		  continue;
	  }

	  MbusRecord record;
	  uint8_t ret = this->parse_data_record(&tg[pos], record);
	  if(!ret) { //error logging is done in parse_data_record
	    pos=0;
	    ESP_LOGW(TAG, "%llx: Variable payload parsing aborted", this->secondary_address);
	    break;
	  }
	  this->records.push_back(record);
	  pos+=ret;
  } //end while

  if(pos) { ESP_LOGD(TAG, "%llx: Parsing variable payload done, %d records", this->secondary_address, (int) this->records.size()); }

  if( pos > (len-3) ){
	  ESP_LOGW(TAG, "%llx: Overrun while parsing telegram.", this->secondary_address);
	  pos=0;
  }

  //manufacturer-specific data
  if(pos && (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME)){
	  ESP_LOGW(TAG, "%llx: Multitelegram readout not supported, some data may be unavailable.", this->secondary_address);
  }


  if(pos && ( (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC)
    || (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME) ) ) {
    pos++;
    uint16_t mfg_specific_begin = pos;
    std::string manufacturer_specific_data;
    for(pos=len-3;pos>=mfg_specific_begin;pos--){
	  sprintf(buf, "%02X ", tg[pos]);
      manufacturer_specific_data += buf;
    }
    ESP_LOGD(TAG, "%llx: Manufacturer-specific data: %s", this->secondary_address, manufacturer_specific_data.c_str());
  }

  //tg[len-2] checksum (verified by statemachine)
  //tg[len-1] stop byte
}

/* uint8_t Mbus::parse_data_record(uint8_t* tg, MbusRecord& record_out):
 *
 * Parse a single variable-length Data Record and render its value as a float.
 *
 * tg: pointer to the Data Record's DIF
 * record_out: filled in with the decoded function/storage/tariff/subunit/
 *             vif_vife/value on success
 *
 * Returns 0 on error (logs the error), length of parsed data otherwise.
 * */

uint8_t Mbus::parse_data_record(uint8_t* tg, MbusRecord& record_out){
	
	uint8_t pos = 0;
	
	uint64_t storage = 0;
	uint32_t tariff = 0;
	uint16_t subunit = 0;
	enum MbusDIFFunction function;
	enum MbusDIFDatatype datatype;
	
	//DIF
	bool extension_flag = false;
	if(tg[pos] & MBUS_DIF_EXTENSION_MASK) extension_flag = true;
	function = (MbusDIFFunction) (tg[pos] & MBUS_DIF_FUNCTION_MASK);
	datatype = (MbusDIFDatatype) (tg[pos] & MBUS_DIF_DATATYPE_MASK);
	if(tg[pos] & MBUS_DIF_STORAGE_MASK) storage = 1;
	pos++;
	
	//DIFE
	uint8_t dife_count = 0;
	while(extension_flag){
		dife_count++;
		if(dife_count > MBUS_DIFE_MAX){
			ESP_LOGE(TAG, "Too many DIFE fields");
			return 0;
		}
		extension_flag = tg[pos] & MBUS_DIFE_EXTENSION_MASK;
		tariff += ( ( tg[pos] & MBUS_DIFE_TARIFF_MASK ) >> 4 ) << ( (dife_count-1) * 2 );
		subunit += ( ( tg[pos] & MBUS_DIFE_SUBUNIT_MASK ) >> 6 ) << (dife_count-1);
		storage += ( tg[pos] & MBUS_DIFE_STORAGE_MASK ) << ( ( (dife_count-1) * 4) + 1 );
		pos++;
	}

	//VIF + VIFE
	
	/* Due to the complexities involved with multiple coexisting VIF+VIFE schemes and
	 * their possible future extensions, the approach taken is to represent VIF
	 * and the first 7 VIFEs as a single 64-bit integer and ignore the rest. */
	
	uint64_t vif_vife = 0;
	extension_flag = true;
	uint8_t vife_count = 0;
	
	while(extension_flag){
		vife_count++;
		if(vife_count > MBUS_VIFE_MAX){
			ESP_LOGW(TAG, "Too many VIFE fields.");
			return 0;
		}
		if(vife_count > 8){
			ESP_LOGW(TAG, "Too many VIFE fields, ignoring.");
		} else {
			vif_vife = vif_vife << 8;
			vif_vife |= (uint64_t) tg[pos];
		}
		extension_flag = tg[pos] & MBUS_VIFE_EXTENSION_MASK;
		pos++;
	}
	
	//Datatype-dependent value parsing
	
	float result = 0;
	uint64_t resultint = 0;	//intermediate, so no need to do excessive float arithmetric
	uint64_t placevalue = 1;

	switch (datatype){
		
	default:
		ESP_LOGE(TAG, "Unknown datatype %d", datatype);
		return 0;

	case MBUS_NO_DATA:
	case MBUS_SELECTION:
		break;
	
	case MBUS_SPECIAL:
		ESP_LOGE(TAG, "Unexpected SPECIAL FUNCTION datatype %d", datatype);
		return 0;
	
	case MBUS_VARIABLE_LEN:
		ESP_LOGW(TAG, "VARIABLE LENGTH datatype len = %d, decoding not yet supported.", tg[pos]);
		pos++; pos+=tg[pos-1];
		break;
		
	case MBUS_REAL:
		pos+=4;
		ESP_LOGW(TAG, "REAL datatype, decoding not yet supported.");
		break;
	
	case MBUS_INT_64BIT:
		resultint = placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
	case MBUS_INT_48BIT:
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
	case MBUS_INT_32BIT:
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
	case MBUS_INT_24BIT:
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
	case MBUS_INT_16BIT:
		resultint |= placevalue * tg[pos];
		pos++;
		placevalue <<= 8;
		
	case MBUS_INT_8BIT:
		resultint |= placevalue * tg[pos];
		pos++;
		
		//M-Bus INTn datatypes are signed (two's complement, per EN 13757-3).
		//resultint above was built as a plain unsigned accumulator, so it must
		//be sign-extended here based on the actual bit width of "datatype" -
		//otherwise a negative value (e.g. delta-T or power near/below zero)
		//comes out as a huge positive number.
		{
			uint8_t bits;
			switch (datatype) {
				case MBUS_INT_8BIT:  bits = 8;  break;
				case MBUS_INT_16BIT: bits = 16; break;
				case MBUS_INT_24BIT: bits = 24; break;
				case MBUS_INT_32BIT: bits = 32; break;
				case MBUS_INT_48BIT: bits = 48; break;
				default:              bits = 64; break; // MBUS_INT_64BIT
			}
			int64_t signed_result = (int64_t) resultint;
			if (bits < 64) {
				uint64_t sign_bit = (uint64_t)1 << (bits - 1);
				if (resultint & sign_bit) {
					signed_result = (int64_t) resultint - ((int64_t)1 << bits);
				}
			}
			result = (float) signed_result;
		}
		break;
	
	case MBUS_BCD12:
		resultint = placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		placevalue *= 100;
		
		resultint += placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		placevalue *= 100;
		
	case MBUS_BCD8:
		resultint += placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		placevalue *= 100;
	
	case MBUS_BCD6:
		resultint += placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		placevalue *= 100;
		
	case MBUS_BCD4:
		resultint += placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		placevalue *= 100;
		
	case MBUS_BCD2:
		resultint += placevalue * ( (tg[pos]&0x0f)+(10*((tg[pos]>>4)&0x0f)) );
		pos++;
		result = (float) resultint;
		break;
		
	}

	ESP_LOGD(TAG, "function: %s, datatype: %s, storage: %lld, tariff: %d, subunit: %d, VIF(E): 0x%llX, value: %lld",
	MbusDIFFunctionToStr(function), MbusDIFDatatypeToStr(datatype), storage, tariff, subunit, vif_vife, resultint);
	
	record_out.function = function;
	record_out.storage = storage;
	record_out.tariff = tariff;
	record_out.subunit = subunit;
	record_out.vif_vife = vif_vife;
	record_out.value = result;
	
	return pos;
}
	
}  // namespace mbus
}  // namespace esphome

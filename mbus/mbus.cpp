#include "mbus.h"
#include "esphome/core/application.h"
//#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mbus {

static const char *const TAG = "mbus";

/* ESPHome code guide says "Use of static variables within component/platform
 *  classes is not permitted, as this is likely to cause problems when multiple
 *  instances of the component/platform are created".
 * Here, however, this is the explicit goal: to share mbus_uart_locked_ between
 * all instances, providing mutually exclusive access to the underlying UART.
 * This is necessary as communication may take a long time (up to a second or more)
 * to be completed, so update() of another Mbus instance may be called before the
 * transaction finishes, causing the transaction to be clobbered.
 * */ 
static bool mbus_uart_locked_;

/* calculate checksum for "long-type" mbus frame
 * */
uint8_t Mbus::mbus_checksum(const uint8_t* data) {
	uint8_t len;
	uint8_t ret;
	if(data[0] != 0x68) return 0;	//not MBUS_FRAME_TYPE_LONG
	len=data[1]+6;
	
	ret=data[4]; //control
	ret+=data[5]; //address
	ret+=data[6]; //control_information
	
	
	for(int i=7;i<=len-3;i++)ret+=data[i];	//payload
	
	return ret;
}

void Mbus::setup() {
	uint32_t tbit_us;
	//statemachine
 mbus_uart_locked_ = false;
 this->mbus_state_ = MBUS_STATE_IDLE;
 this->mbus_update_due_ = false;
 
 //baudrate-dependent timeouts
 tbit_us = 1000000 / this->parent_->get_baud_rate();
 this->mbus_timeout_short_ = ( (330 + 11) * tbit_us ) / 1000;
 this->mbus_timeout_short_ += 150;
 this->mbus_timeout_long_ = ( (330 + 11 + (11*512)) * tbit_us ) / 1000;
 this->mbus_timeout_long_ += 150;
 
 //prepare "select secondary address" frame
 for(int i=0; i<=mbus_select_frame_len_-1;i++) this->mbus_select_frame_[i]=mbus_select_frame_raw_[i];
 this->mbus_select_frame_[7] = (uint8_t) ( this->secondary_address >> (4*8) );
 this->mbus_select_frame_[8] = (uint8_t) ( this->secondary_address >> (5*8) );
 this->mbus_select_frame_[9] = (uint8_t) ( this->secondary_address >> (6*8) );
 this->mbus_select_frame_[10] = (uint8_t) ( this->secondary_address >> (7*8) );
 this->mbus_select_frame_[11] = (uint8_t) ( this->secondary_address >> (3*8) );
 this->mbus_select_frame_[12] = (uint8_t) ( this->secondary_address >> (2*8) );
 this->mbus_select_frame_[13] = (uint8_t) ( this->secondary_address >> (1*8) );
 this->mbus_select_frame_[14] = (uint8_t) ( this->secondary_address >> (0*8) );
 this->mbus_select_frame_[15] = mbus_checksum(mbus_select_frame_);

 //signal to sensors
 this->telegram_count=0;
}

void Mbus::loop() {
  uint32_t now_;
  
  now_=App.get_loop_component_start_time();
  
  switch(this->mbus_state_){
	  default:
	  ESP_LOGE(TAG, "%llx STATEMACHINE IN UNKNOWN STATE, RESETTING", this->secondary_address);
	  this->mbus_state_ = MBUS_STATE_IDLE;
	  mbus_uart_locked_ = false;
	  break;
	  
	  case MBUS_STATE_IDLE:
	  if(this->mbus_update_due_){
		  this->mbus_update_due_ = false;
		  this->mbus_state_ = MBUS_STATE_AWAIT_LOCK;
      }
	  break;
	  
	  //wait until no other mbus instances are using the uart, then locking it for ourselves
	  case MBUS_STATE_AWAIT_LOCK:
	  if(mbus_uart_locked_)break;
	  mbus_uart_locked_ = true;
	  this->mbus_retry_count_ = mbus_max_retries_;
	  this->mbus_state_ = MBUS_STATE_BUS_RESET_PRE;
	  break;
	  
	  //resetting the bus
	  case MBUS_STATE_BUS_RESET_PRE:
	  this->write_array(mbus_reset_frame_, mbus_reset_frame_len_);
	  ESP_LOGD(TAG, " %llx: sending first bus reset", this->secondary_address);
	  this->mbus_timer_ = now_;
	  this->mbus_state_ = MBUS_STATE_BUS_RESET;
	  break;
	  
	  case MBUS_STATE_BUS_RESET:
	  if(now_ < this->mbus_timer_ + this->mbus_timeout_short_) break;
	  //sending another reset, as the first may be lost
	  ESP_LOGD(TAG, " %llx: sending second bus reset", this->secondary_address);
	  this->write_array(mbus_reset_frame_, mbus_reset_frame_len_);
	  this->mbus_timer_ = now_;
	  this->mbus_state_ = MBUS_STATE_BUS_RESET_2;
	  break;
	  
	  case MBUS_STATE_BUS_RESET_2:
	  if(now_ < this->mbus_timer_ + this->mbus_timeout_short_) break;
	  //purge rx buffer
	  ESP_LOGD(TAG, " %llx: purging rx buffer", this->secondary_address);
	  while(this->available()) this->read_array( this->telegram, 1 ) ;
	  //select device on bus
	  ESP_LOGD(TAG, " %llx: sending SELECT SECONDARY ADDRESS command", this->secondary_address);
	  this->write_array(this->mbus_select_frame_, mbus_select_frame_len_);
	  this->mbus_timer_ = now_;
	  this->mbus_state_ = MBUS_STATE_AWAIT_SELSCT_SA;
	  break;
	  
	  case MBUS_STATE_AWAIT_SELSCT_SA:
	  if(!this->available()){
		  if(now_ < this->mbus_timer_ + this->mbus_timeout_short_) break;
		  else {
			  ESP_LOGE(TAG, "%llx: Timeout while waiting for ACK", this->secondary_address);
			  this->mbus_state_ = MBUS_STATE_RETRY;
			  break;
		  }
	  } else {
		  this->read_byte(this->telegram);
		  if( this->telegram[0] != mbus_ack_ ){
			  ESP_LOGE(TAG, "%llx: Collision while waiting for ACK", this->secondary_address);
			  this->mbus_state_ = MBUS_STATE_RETRY;
			  break;
		  }
		  this->mbus_timer_ = now_;
		  this->mbus_state_ = MBUS_STATE_AWAIT_SELSCT_SA_2;
		  break;
	  }
	  
	  //if we got acknowledge, that does not exclude further collision
	  case MBUS_STATE_AWAIT_SELSCT_SA_2:
	  if(this->available()){
		  ESP_LOGE(TAG, "%llx: Collision while waiting for ACK", this->secondary_address);
		  this->mbus_state_ = MBUS_STATE_RETRY;
		  break;
	  }
	  if(now_ < this->mbus_timer_ + this->mbus_timeout_short_) break;
	  //ack without collision --> request data
	  ESP_LOGD(TAG, " %llx: sending REQUEST DATA command", this->secondary_address);
	  this->write_array(mbus_request_frame_, mbus_request_frame_len_);
	  this->mbus_timer_ = now_;
	  this->mbus_state_ = MBUS_STATE_AWAIT_HEADER;
	  break;
	  
	  case MBUS_STATE_AWAIT_HEADER:
	  if(now_ > this->mbus_timer_ + this->mbus_timeout_long_){
		  ESP_LOGE(TAG, "%llx: Timeout while waiting for header", this->secondary_address);
		  this->mbus_state_ = MBUS_STATE_RETRY;
	  }
	  if(this->available() < 3) break;
	  this->read_array(this->telegram, 3);
	  if( (this->telegram[0] != mbus_long_frame_) || (this->telegram[1] != this->telegram[2]) ){
		  ESP_LOGE(TAG, "%llx: Invalid header %02hhX %02hhX %02hhX ", this->secondary_address, this->telegram[0], this->telegram[1], this->telegram[2]);
		  this->mbus_state_ = MBUS_STATE_RETRY_WAIT;
		  break;
	  }
	  this->mbus_telegram_len_ = this->telegram[1] + 3;
	  ESP_LOGD(TAG, " %llx: header received, len: %d", this->secondary_address, this->telegram[1]);
	  //got header, getting the rest of frame
	  this->mbus_state_ = MBUS_STATE_AWAIT_DATA;
	  break;
	  
	  case MBUS_STATE_AWAIT_DATA:
	  if(now_ > this->mbus_timer_ + this->mbus_timeout_long_){
		  ESP_LOGE(TAG, "%llx: Timeout while waiting for data", this->secondary_address);
		  this->mbus_state_ = MBUS_STATE_RETRY;
	  }
	  if(this->available() < this->mbus_telegram_len_) break;
	  //entire response received, checking checksum
	  this->read_array(&(this->telegram[3]), this->mbus_telegram_len_);
	  ESP_LOGD(TAG, " %llx: data received", this->secondary_address);
	  if(this->telegram[this->mbus_telegram_len_+1] != mbus_checksum(this->telegram)) {
		  ESP_LOGE(TAG, "%llx: Invalid checksum, expected %d", this->secondary_address, mbus_checksum(this->telegram));
		  this->mbus_state_ = MBUS_STATE_RETRY_WAIT;
		  break;
	  }
	  //checksum ok - decode the telegram once, here, for all sensors to read
	  this->parse_telegram();
	  //releasing uart
	  mbus_uart_locked_ = false;
	  this->mbus_state_ = MBUS_STATE_IDLE;
	  this->telegram_count++;
	  break;
	  
	  case MBUS_STATE_RETRY_WAIT:
	  if(now_ > this->mbus_timer_ + this->mbus_timeout_long_){
		  this->mbus_state_ = MBUS_STATE_RETRY;
	  }
	  break;
	  
	  case MBUS_STATE_RETRY:
	  this->mbus_retry_count_ --;
	  if(this->mbus_retry_count_) {
		  this->mbus_state_ = MBUS_STATE_BUS_RESET_PRE;
		  ESP_LOGD(TAG, " %llx: retrying", this->secondary_address);
		  break;
	  }
	  ESP_LOGE(TAG, " %llx: Retries exhausted, aborting.", this->secondary_address);
	  mbus_uart_locked_ = false;
	  this->mbus_state_ = MBUS_STATE_IDLE;
	  break;
	  
	  
	  } //end switch
}

void Mbus::update() {
 ESP_LOGD(TAG, "update(): %llx, locked: %s", this->secondary_address, YESNO(mbus_uart_locked_));
 this->mbus_update_due_ = true;
}

void Mbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Mbus:");
  ESP_LOGCONFIG(TAG, "  Secondary address: %llX", this->secondary_address);
  
}

float Mbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}



}  // namespace mbus
}  // namespace esphome

#ifndef NAVATICSCANINTERFACE_CANMESSAGE_H
#define NAVATICSCANINTERFACE_CANMESSAGE_H

#include <vector>

namespace navatics_can_interface{

class CANMessage{

  public:
  // Constructor
  CANMessage(){} // constructor

  // complete constructor that will check message validity
  CANMessage(uint16_t _std_id, uint8_t _len, std::vector<uint8_t> _data){
    if (this->valid(_std_id, _len, _data)){
      this->std_id = _std_id;
      this->len = _len;
      this->data = std::vector<uint8_t>(_data.begin(), _data.begin()+this->len);
    }
  } // constructor

  // message parameters
  uint16_t std_id = 0x0000;
  uint8_t len = 0;
  std::vector<uint8_t> data;

  bool valid(){
    this->valid(this->std_id, this->len, this->data);
  } // valid()
  

  private:
  bool valid(uint16_t _std_id, uint8_t _len, std::vector<uint8_t> _data){
    bool is_msg_valid = true;
    if (_std_id > 0x07FF)
      is_msg_valid = false;
    // check and get length
    if (_len > 8)
      is_msg_valid = false;
    else if (_data.size() != _len)
      is_msg_valid = false;
    return is_msg_valid;
  } // valid(_std_id, _len, _data)
  

}; //class CANMessage

} // namespace navatics_can_interface

#endif // NAVATICSCANINTERFACE_CANMESSAGE_H


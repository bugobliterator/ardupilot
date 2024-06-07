#include "AP_DroneCAN_File_Client.h"

void AP_DroneCAN_File_Client::handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg){
    return;   
}

bool AP_DroneCAN_File_Client::write_async(uint8_t data[], uint16_t data_len, FileRequestCb& file_request_cb) {
    return true;
}

bool AP_DroneCAN_File_Client::open_async(const char *name, uint16_t flags, FileRequestCb& file_request_cb) {
    return true;
}
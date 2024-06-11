#include "AP_DroneCAN_File_Client.h"

void AP_DroneCAN_File_Client::handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg){
    // uavcan_protocol_file_WriteRequest req;
    return;   
}

bool AP_DroneCAN_File_Client::write_async(uint8_t data[], uint16_t data_len, FileRequestCb *cb) {
    uavcan_protocol_file_WriteRequest req;
    req.data.len = data_len;
    memcpy(req.data.data, data, sizeof(req.data.data));
    file_request_cb = cb;

    _write_client.request(,req);

    return true;
}

bool AP_DroneCAN_File_Client::open_async(const char *name, uint16_t flags, FileRequestCb *cb) {
    // uavcan_protocol_file_ReadRequest req;

    file_request_cb = cb;

    return true;
}
#include "AP_DroneCAN_File_Client.h"
#include <AP_FileSystem/AP_Filesystem.h>

bool AP_DroneCAN_File_Client::init(uint8_t server_node_id)
{
    _server_node_id = server_node_id;
}

void AP_DroneCAN_File_Client::handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg){
    // uavcan_protocol_file_WriteRequest req;
    return;   
}

int AP_DroneCAN_File_Client::open(const char *path, int mode)
{
    if (_server_node_id == 0) {
        errno = ENODEV;
        return -1;
    }
    WITH_SEMAPHORE(_sem);

    if (is_opened) {
        errno = EBUSY;
        return -1;
    }

    // we only support readonly and writeonly modes
    if (mode == O_RDONLY) {
        is_reading = true;
    } else if (mode == O_WRONLY) {
        is_reading = false;
    } else {
        errno = EINVAL;
        return -1;
    }
    _total_transaction = 0;
    // make sure we don't have a hangover from last close call
    file_request_cb = nullptr;
    is_opened = true;

    return 0;
}

bool AP_DroneCAN_File_Client::write_async(uint8_t data[], uint16_t data_len, FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem);

    uavcan_protocol_file_WriteRequest req;

    req.offset = _total_transaction;
    req.data.len = data_len;
    memcpy(req.data.data, data, sizeof(req.data.data));
    file_request_cb = cb;
    _write_client.request(_server_node_id, req);

    // increment total transaction
    _total_transaction += data_len;
    return true;
}

void AP_DroneCAN_File_Client::close_async(FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    if (!is_reading) {
        // mark the end of the file by sending an empty write request with the offset set to the total transaction
        uavcan_protocol_file_WriteRequest req;
        req.offset = _total_transaction;
        req.data.len = 0;
        file_request_cb = cb;
        _write_client.request(_server_node_id, req);
        _total_transaction = 0;
    }
}

void handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg)
{
    if (file_request_cb) {
        file_request_cb(&_ap_dronecan, msg.error.value);
    }
}

bool AP_DroneCAN_File_Client::read_async(uint16_t offset, uint16_t len, FileRequestCb *cb) {
    if (_server_node_id == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem);

}
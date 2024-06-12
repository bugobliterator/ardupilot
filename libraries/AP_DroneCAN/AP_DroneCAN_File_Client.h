#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include "AP_Canard_iface.h"
#include <dronecan_msgs.h>

class AP_DroneCAN;
//Forward declaring classes
class AP_DroneCAN_File_Client
{
public:
    AP_DroneCAN_File_Client(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_File_Client);

    FUNCTOR_TYPEDEF(FileRequestCb, void, AP_DroneCAN*, const int16_t);

    bool init(uint8_t _server_node_id);

    bool open(const char *path, uint16_t mode);

    // sends the write request to the server 
    bool write_async(uint8_t data[], uint16_t data_len, FileRequestCb *cb);

    // sends the read request to the server
    bool read_async(uint16_t offset, uint16_t len, FileRequestCb *cb);

    // close the file
    bool close();

private:
    HAL_Semaphore storage_sem;
    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    // gets called at receive of write response
    void handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg);
    Canard::ObjCallback<AP_DroneCAN_File_Client, uavcan_protocol_file_WriteResponse> file_wr_cb{this, &AP_DroneCAN_File_Client::handle_write_response};
    Canard::Client<uavcan_protocol_file_WriteResponse> _write_client{_canard_iface, file_wr_cb};;


    FileRequestCb *file_request_cb;

    uint8_t _server_node_id;

    uint32_t _total_transaction;

    bool is_opened;

    HAL_Semaphore _sem;
};
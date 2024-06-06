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

    HAL_Semaphore storage_sem;
    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    // gets called at receive of write response
    void handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg);
    Canard::ObjCallback<AP_DroneCAN_File_Client, uavcan_protocol_file_WriteResponse> file_wr_cb{this, &AP_DroneCAN_File_Client::handle_write_response};
    Canard::Client<uavcan_protocol_file_WriteResponse> _write_client;

public:
    AP_DroneCAN_File_Client(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t file_server_node_id);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_File_Client);

    FUNCTOR_TYPEDEF(FileRequestCb, void, AP_DroneCAN*, const int16_t);

    // sends request to open a file on the server
    bool open_async(const char *name, uint16_t flags, FileRequestCb& file_request_cb);

    // sends the write request to the server 
    bool write_async(uint8_t data[], uint16_t data_len, FileRequestCb& file_request_cb);
};
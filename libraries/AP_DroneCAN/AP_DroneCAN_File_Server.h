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

#define MAX_NUM_CLIENTS 5

class AP_DroneCAN;
//Forward declaring classes
class AP_DroneCAN_File_Server
{
public:
    AP_DroneCAN_File_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface);

    bool init();

    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_File_Server);

private:
    HAL_Semaphore storage_sem;
    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    void handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest& req);
    Canard::ObjCallback<AP_DroneCAN_File_Server, uavcan_protocol_file_WriteRequest> file_write_cb{this, &AP_DroneCAN_File_Server::handle_write_request};
    Canard::Server<uavcan_protocol_file_WriteRequest> _write_server{_canard_iface, file_write_cb};

    void handle_read_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadRequest& req);
    Canard::ObjCallback<AP_DroneCAN_File_Server, uavcan_protocol_file_ReadRequest> file_read_cb{this, &AP_DroneCAN_File_Server::handle_read_request};
    Canard::Server<uavcan_protocol_file_ReadRequest> _read_server{_canard_iface, file_read_cb};

    struct FileClient {
        int fd = -1;
        uavcan_protocol_file_Path path;
        uint8_t open_mode;
        uint8_t client_node_id;
        uint32_t last_request_time_ms;
    } _clients[MAX_NUM_CLIENTS];

    FileClient* get_free_client(const uavcan_protocol_file_Path &path, uint8_t client_node_id);

};
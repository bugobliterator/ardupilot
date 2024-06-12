#include <AP_Filesystem/AP_Filesystem.h>
#include "AP_DroneCAN_File_Server.h"


AP_DroneCAN_File_Server::FileClient* AP_DroneCAN_File_Server::get_free_client(const uavcan_protocol_file_Path &path, uint8_t client_node_id)
{
    // check if the file is already open
    for (int i = 0; i < MAX_NUM_CLIENTS; i++) {
        if (_clients[i].fd >= 0 && _clients[i].path.path.len == path.path.len && memcmp(_clients[i].path.path.data, path.path.data, path.path.len) == 0) {
            return &_clients[i];
        }
    }

    for (int i = 0; i < MAX_NUM_CLIENTS; i++) {
        if (_clients[i].fd < 0) {
            memcpy(&_clients[i].path, &path, sizeof(_clients[i].path));
            return &_clients[i];
        }
    }
    return nullptr;
}

void AP_DroneCAN_File_Server::handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest& req){
    uavcan_protocol_file_WriteResponse rsp;

    // get a free client record slot
    FileClient* client = get_free_client(req.path, transfer.source_node_id);
    if (client == nullptr) {
        // we don't have any free slots, respond with stream unavailable
        rsp.error.value = ENOSR;
        _write_server.respond(transfer, rsp);
        return;
    }

    if (client->open_mode != O_WRONLY && client->fd >= 0) {
        // file is not open for writing, reopen
        AP::FS().close(client->fd);
        client->fd = -1;
    }

    // Try opening the file
    if (client->fd < 0) {
        // make sure the path is null terminated
        if (strnlen((const char*)req.path.path.data, req.path.path.len) == req.path.path.len) {
            rsp.error.value = ENAMETOOLONG;
            _write_server.respond(transfer, rsp);
            return;
        }
        // open the file
        client->fd = AP::FS().open((const char*)req.path.path.data, O_WRONLY);
    }

    // Check if we successfully opened the file
    if (client->fd < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = client->fd;
        }
        _write_server.respond(transfer, rsp); 
        return;
    }

    // check if it is final call to finish the write
    if (req.data.len == 0 && req.offset != 0) {
        AP::FS().close(client->fd);
        client->fd = -1;
        _write_server.respond(transfer, rsp);
        return;
    }

    // seek to the requested offset
    int ret = AP::FS().lseek(client->fd, req.offset, SEEK_SET);
    if (ret < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = ret;
        }
        _write_server.respond(transfer, rsp); 
        return;
    }
    ret = AP::FS().write(client->fd, req.data.data, req.data.len);
    if (ret < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = ret;
        }
    }
    if (ret < req.data.len) {
        // we didn't write all the data
        rsp.error.value = EAGAIN;
    }
    _write_server.respond(transfer, rsp);
}


void AP_DroneCAN_File_Server::handle_read_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadRequest& req){
    uavcan_protocol_file_ReadResponse rsp;
    int fd = AP::FS().open("@ROMFS/test.123", O_RDONLY, false);
    if (fd<0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = fd;
        }
        _read_server.respond(transfer, rsp); 
        return;
    }
    int ret = AP::FS().read(fd, rsp.data.data, req.path.path.len);
    // int ret = AP::FS().read(fd, req.path.path.data, req.path.path.len);   // Giving const error
    if (ret < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = ret;
        }
    }
    _read_server.respond(transfer, rsp);  
}

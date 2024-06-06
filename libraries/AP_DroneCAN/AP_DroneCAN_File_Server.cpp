#include <AP_Filesystem/AP_Filesystem.h>
#include "AP_DroneCAN_File_Server.h"

void AP_DroneCAN_File_Server::handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest& req){
    uavcan_protocol_file_WriteResponse rsp;
    int fd = AP::FS().open("@ROMFS/test.123", O_CREAT, 0);
    if (fd<0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = fd;
        }
        _write_server.respond(transfer, rsp); 
        return;
    }
    int ret = AP::FS().write(fd, req.data.data, req.data.len);
    if (ret < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = ret;
        }
    }
    _write_server.respond(transfer, rsp);
}


void AP_DroneCAN_File_Server::handle_read_request(const CanardRxTransfer& transfer, uavcan_protocol_file_ReadRequest& req){
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
    if (ret < 0) {
        if (errno != 0) {
            rsp.error.value = errno;
        } else {
            rsp.error.value = ret;
        }
    }
    _read_server.respond(transfer, rsp);  
}

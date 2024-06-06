#include <AP_Filesystem/AP_Filesystem.h>
#include "AP_DroneCAN_File_Server.h"

void AP_DroneCAN_File_Server::handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest& req)
{
    int fd = AP::FS().open("@ROMFS/test.123", O_CREAT, 0);
    if (fd<0) {
        rsp.error.value = UAVCAN_PROTOCOL_FILE_ERROR_UNKNOWN_ERROR;
        _write_server.respond(transfer, rsp); 
        return;
    }
    int ret = AP::FS().write(fd, req.data.data, req.data.len);
    if (ret < 0) {
        if (errno != 0) {
            rsp.error = errno;
        } else {
            rsp.error = ret;
        }
    }
    _write_server.respond(transfer, rsp);    
}

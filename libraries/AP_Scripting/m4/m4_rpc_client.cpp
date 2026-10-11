#include "m4_rpc_client.h"

#include <errno.h>
#include <string.h>
#include "m4_port.h"

namespace ScriptingM4 {

RpcClient rpc;

RpcWriter &RpcClient::begin(uint16_t method, bool notify)
{
    _method = method;
    _notify = notify;
    _count = 0;
    _reply_length = 0;
    memset(&_reply_header, 0, sizeof(_reply_header));
    _writer.begin(_request, sizeof(_request), sizeof(CallHeader));
    return _writer;
}

int RpcClient::invoke()
{
    if (!_writer.ok()) {
        return -EMSGSIZE;
    }
    CallHeader call;
    fill_header(call.header, _notify ? MsgType::Notify : MsgType::Call, _writer.length(), 0);
    call.method = _method;
    call.argc = _writer.count();
    call.reserved = 0;
    memcpy(_request, &call, sizeof(call));
    const int32_t ret = port_call(_request, _writer.length(), _reply, sizeof(_reply));
    if (ret < 0) {
        return ret;
    }
    if (_notify) {
        return 0;
    }
    if (ret < int32_t(sizeof(ReplyHeader))) {
        return -EBADMSG;
    }
    _reply_length = uint16_t(ret);
    memcpy(&_reply_header, _reply, sizeof(_reply_header));
    if (!decode_values(&_reply[sizeof(ReplyHeader)], uint16_t(ret) - sizeof(ReplyHeader),
                       _reply_header.count, _values)) {
        return -EBADMSG;
    }
    _count = _reply_header.count;
    return 0;
}

bool RpcClient::copy_reply(uint8_t *buf, uint16_t size, RpcValue *values, uint8_t &count) const
{
    if (_reply_length < sizeof(ReplyHeader)) {
        return false;
    }
    const uint16_t length = uint16_t(_reply_length - sizeof(ReplyHeader));
    if (length > size) {
        return false;
    }
    memcpy(buf, &_reply[sizeof(ReplyHeader)], length);
    count = _count;
    return decode_values(buf, length, count, values);
}

}  // namespace ScriptingM4

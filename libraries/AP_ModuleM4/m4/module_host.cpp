#include "module_host.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <AP_Module/Bindings.h>

namespace ModuleM4 {

using AP_Module::Image;

// the M4's own load failure details, after the PR's "<file> load failed: "
static const char NO_ROOM[] = "insufficient M4 RAM to load";
static const char CHECKSUM_MISMATCH[] = "checksum mismatch";
static const char CATALOGUE_MISMATCH[] = "local binding catalogue mismatch";

// text formatting without stdio (not on the privileged import list): appends up to max characters
static void append(char *text, uint32_t size, uint32_t &length, const char *value, uint32_t max = UINT32_MAX)
{
    for (uint32_t i = 0; value[i] != 0 && i < max && length + 1 < size; i++) {
        text[length++] = value[i];
    }
    text[length] = 0;
}

static void append_hex(char *text, uint32_t size, uint32_t &length, uint32_t value)
{
    char digits[9];
    for (uint8_t i = 0; i < 8; i++) {
        digits[i] = "0123456789abcdef"[(value >> (28U - 4U * i)) & 0xFU];
    }
    digits[8] = 0;
    append(text, size, length, digits);
}

static void copy_terminated(char *out, uint32_t size, const char *in, uint32_t in_size)
{
    uint32_t i = 0;
    for (; i + 1 < size && i < in_size && in[i] != 0; i++) {
        out[i] = in[i];
    }
    out[i] = 0;
}

static uint32_t read_u32(const uint8_t *p)
{
    uint32_t value;
    memcpy(&value, p, sizeof(value));
    return value;
}

static uint64_t read_u64(const uint8_t *p)
{
    uint64_t value;
    memcpy(&value, p, sizeof(value));
    return value;
}

// Resolve: the hashes check_imports() visits, in import order, written into the request
struct HashCollector {
    uint8_t *hashes;
    uint32_t count;
};

static uint32_t collect_hash(void *context, uint64_t hash)
{
    auto &collector = *static_cast<HashCollector *>(context);
    if (collector.count < RESOLVE_MAX) {
        memcpy(collector.hashes + collector.count * sizeof(uint64_t), &hash, sizeof(hash));
    }
    collector.count++;
    return 0;
}

// the M7's answer of the Resolve round: hashes in the request, ids in the reply. fail_at makes
// the call of that index answer UINT32_MAX, so check_imports() names that import
struct ResolvedTable {
    const uint8_t *hashes;
    const uint8_t *ids;
    uint32_t count;
    uint32_t calls;
    uint32_t fail_at;
};

static uint32_t resolved_id(void *context, uint64_t hash)
{
    auto &table = *static_cast<ResolvedTable *>(context);
    if (table.calls++ == table.fail_at) {
        return UINT32_MAX;
    }
    for (uint32_t i = 0; i < table.count; i++) {
        if (read_u64(table.hashes + i * sizeof(uint64_t)) == hash) {
            return read_u32(table.ids + i * sizeof(uint32_t));
        }
    }
    return UINT32_MAX;
}

static ResolvedTable resolved_table(const ModuleLink &link, uint32_t count)
{
    return ResolvedTable { link.request + offsetof(ResolveRequest, hashes), link.reply + offsetof(ResolveReply, ids),
                           count, 0, UINT32_MAX };
}

// the M4's own answer for a local hash (the catalogue is sorted by hash); false if not local
static bool local_import(uint64_t hash, uint32_t &id)
{
    uint32_t low = 0, high = AP_Module::M4_LOCAL_IMPORT_COUNT;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2U;
        const AP_Module::LocalImport &entry = AP_Module::M4_LOCAL_IMPORTS[middle];
        if (entry.hash == hash) {
            id = entry.id;
            return true;
        }
        if (entry.hash < hash) {
            low = middle + 1U;
        } else {
            high = middle;
        }
    }
    return false;
}

void ModuleHost::init(const ModuleHostPlatform &platform)
{
    _platform = platform;
    _step = Step::Config;
    _calling = false;
    _start_phase = 0;
    _failure[0] = 0;
    _name[0] = 0;
    _closing = false;
    _length = 0;
    _module.state.store(0);
}

void ModuleHost::post(uint16_t length)
{
    ModuleLink &link = *_platform.link;
    link.request_len = length;
    link.state.store(ModuleLink::Posted);
    _calling = true;
}

bool ModuleHost::completed(int32_t &result)
{
    ModuleLink &link = *_platform.link;
    if (!_calling || link.state.load() != ModuleLink::Done) {
        return false;
    }
    result = link.result;
    link.state.store(ModuleLink::Idle);
    _calling = false;
    return true;
}

// the reply of the call that completed with result: a whole SystemReply of method, Ok, of at
// least minimum bytes
bool ModuleHost::system_reply(int32_t result, SystemMethod method, uint16_t minimum, SystemReplyHeader &reply) const
{
    if (result < int32_t(sizeof(SystemReplyHeader)) || result < int32_t(minimum) || result > MAX_FRAME) {
        return false;
    }
    memcpy(&reply, _platform.link->reply, sizeof(reply));
    return reply.header.type == uint8_t(MsgType::SystemReply) && reply.method == uint16_t(method) &&
           reply.transport == uint8_t(ReplyStatus::Ok);
}

void ModuleHost::post_system_call(SystemMethod method, uint16_t length)
{
    SystemCallHeader call {};
    fill_header(call.header, MsgType::SystemCall, length, 0);
    call.method = uint16_t(method);
    memcpy(_platform.link->request, &call, sizeof(call));
    post(length);
}

void ModuleHost::post_module_state(ModuleStateKind kind)
{
    ModuleStateNotify notify {};
    fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
    notify.call.method = uint16_t(SystemMethod::ModuleState);
    notify.call.flags = CALL_FLAG_NOTIFY;
    notify.module = _module.index;
    notify.state = uint8_t(kind);
    notify.rx_size = _image.code_size();
    notify.rw_size = _image.rw_memory_size();
    notify.heap_size = uint32_t(_config.heap_size);
    notify.stack_size = uint32_t(_config.stack_size);
    notify.crc = _image.checksum();
    if (kind == ModuleStateKind::Failed) {
        copy_terminated(notify.text, sizeof(notify.text), _failure, sizeof(_failure));
    }
    memcpy(_platform.link->request, &notify, sizeof(notify));
    post(sizeof(notify));
}

// a failure in the PR's words (Runtime.cpp's fail() reasons): the M7 prints "Module: <reason>"
void ModuleHost::fail(const char *reason)
{
    uint32_t length = 0;
    append(_failure, sizeof(_failure), length, reason);
    _step = Step::Failed;
    if (!_calling) {
        post_module_state(ModuleStateKind::Failed);
    }
}

// "<file> load failed: <detail>"
void ModuleHost::fail_load(const char *detail)
{
    char reason[sizeof(_failure)];
    uint32_t length = 0;
    append(reason, sizeof(reason), length, _name, 31);
    append(reason, sizeof(reason), length, " load failed: ");
    append(reason, sizeof(reason), length, detail);
    fail(reason);
}

void ModuleHost::step()
{
    int32_t result = 0;
    if (_calling && !completed(result)) {
        return;   // the posted call is still on its way
    }
    switch (_step) {
    case Step::Config:
        // the service asked for it at session start; the M7 lists no module before it
        if (!_platform.config(_platform.context, _config)) {
            return;
        }
        if (_platform.unpublish != nullptr) {
            _platform.unpublish(_platform.context);   // e.g. the selftest's window and guard
        }
        post_system_call(SystemMethod::ListModules, sizeof(SystemCallHeader));
        _step = Step::List;
        return;
    case Step::List:
        list_modules(result);
        return;
    case Step::Open: {
        SystemReplyHeader reply;
        if (!system_reply(result, SystemMethod::FsOpen, 0, reply) || reply.result < 0) {
            fail("module open failed");
            return;
        }
        _fd = reply.result;
        _step = Step::Read;
        post_read();
        return;
    }
    case Step::Read:
        read(result);
        return;
    case Step::Parse:
        parse();
        return;
    case Step::Resolve:
        resolve(result);
        return;
    case Step::Place:
        place();
        return;
    case Step::Load:
        load();
        return;
    case Step::Checksum:
        // both are zlib's CRC-32 of the canonical file: the M4's over what it parsed, the M7's
        // over what it served
        if (_image.checksum() != _served_crc || _served != _length) {
            fail_load(CHECKSUM_MISMATCH);
            return;
        }
        _step = Step::Publish;
        return;
    case Step::Publish:
        publish();
        return;
    case Step::Start:
        start();
        return;
    case Step::Running:
    case Step::Failed:
    case Step::Idle:
        return;
    }
}

void ModuleHost::list_modules(int32_t result)
{
    SystemReplyHeader reply;
    const uint16_t entries = offsetof(ListModulesReply, modules);
    if (!system_reply(result, SystemMethod::ListModules, entries, reply)) {
        fail("module directory unavailable");
        return;
    }
    const uint8_t *frame = _platform.link->reply;
    const uint8_t count = frame[offsetof(ListModulesReply, count)];
    if (count == 0) {
        _step = Step::Idle;
        return;
    }
    if (uint32_t(result) < entries + uint32_t(count) * sizeof(ModuleEntry)) {
        fail("module directory unavailable");
        return;
    }
    // the M7 lists at most one module for the M4 (it fails the load of more)
    const uint8_t *entry = frame + entries;
    copy_terminated(_name, sizeof(_name), reinterpret_cast<const char *>(entry + offsetof(ModuleEntry, name)),
                    sizeof(ModuleEntry::name));
    copy_terminated(_path, sizeof(_path), reinterpret_cast<const char *>(entry + offsetof(ModuleEntry, path)),
                    sizeof(ModuleEntry::path));
    _size = read_u32(entry + offsetof(ModuleEntry, size));
    if (_size > _platform.area_size) {
        fail_load(NO_ROOM);
        return;
    }
    FsOpenRequest open {};
    copy_terminated(open.path, sizeof(open.path), _path, sizeof(_path));
    memcpy(_platform.link->request + sizeof(SystemCallHeader), open.path, sizeof(open.path));
    post_system_call(SystemMethod::FsOpen, sizeof(open));
    _step = Step::Open;
}

void ModuleHost::post_read()
{
    const uint32_t left = _size - _length;
    FsReadRequest request {};
    fill_header(request.call.header, MsgType::SystemCall, sizeof(request), 0);
    request.call.method = uint16_t(SystemMethod::FsRead);
    request.fd = _fd;
    request.count = uint16_t(left < FS_READ_MAX ? left : FS_READ_MAX);
    memcpy(_platform.link->request, &request, sizeof(request));
    post(sizeof(request));
}

// staging: the file at the bottom of the module area, in order, up to its listed size; a file
// that ends early is parsed as it is and fails the parse
void ModuleHost::read(int32_t result)
{
    SystemReplyHeader reply;
    if (_closing) {
        FsCloseReply close;
        if (!system_reply(result, SystemMethod::FsClose, sizeof(close), reply) || result != int32_t(sizeof(close)) ||
            reply.result < 0) {
            fail("module read/size/scratch allocation failed");
            return;
        }
        memcpy(&close, _platform.link->reply, sizeof(close));
        _served_crc = close.crc;
        _served = close.served;
        _step = Step::Parse;
        return;
    }
    const uint32_t requested = _size - _length < FS_READ_MAX ? _size - _length : FS_READ_MAX;
    if (!system_reply(result, SystemMethod::FsRead, 0, reply) || reply.result < 0 ||
        uint32_t(reply.result) > requested || uint32_t(result) != sizeof(reply) + uint32_t(reply.result)) {
        fail("module read/size/scratch allocation failed");
        return;
    }
    const uint32_t count = uint32_t(reply.result);
    memcpy(pointer(_platform.area_base + _length), _platform.link->reply + sizeof(reply), count);
    _length += count;
    if (count != 0 && _length < _size) {
        post_read();
        return;
    }
    FsCloseRequest close {};
    fill_header(close.call.header, MsgType::SystemCall, sizeof(close), 0);
    close.call.method = uint16_t(SystemMethod::FsClose);
    close.fd = _fd;
    memcpy(_platform.link->request, &close, sizeof(close));
    post(sizeof(close));
    _closing = true;
}

void ModuleHost::parse()
{
    if (_config.code_size <= 0 || _config.heap_size <= 0 ||
        _config.stack_size <= int32_t(_platform.stack_margin)) {
        fail_load("invalid memory limits");
        return;
    }
    Image::Compatibility compatibility = _platform.compatibility;
    compatibility.stack_limit = uint32_t(_config.stack_size) - _platform.stack_margin;
    if (!_image.parse(pointer(_platform.area_base), _length, uint32_t(_config.code_size), compatibility)) {
        fail_load(_image.parse_error());
        return;
    }
    if (!_image.is_split()) {
        fail_load("invalid module image");
        return;
    }
    HashCollector collector { _platform.link->request + offsetof(ResolveRequest, hashes), 0 };
    (void)_image.check_imports(collect_hash, &collector);
    if (collector.count > RESOLVE_MAX) {
        fail_load(NO_ROOM);
        return;
    }
    _import_count = uint16_t(collector.count);
    const uint16_t count = _import_count;
    memcpy(_platform.link->request + offsetof(ResolveRequest, count), &count, sizeof(count));
    memset(_platform.link->request + offsetof(ResolveRequest, reserved), 0, sizeof(ResolveRequest::reserved));
    post_system_call(SystemMethod::Resolve, uint16_t(offsetof(ResolveRequest, hashes) + count * sizeof(uint64_t)));
    _step = Step::Resolve;
}

void ModuleHost::resolve(int32_t result)
{
    SystemReplyHeader reply;
    const uint32_t length = offsetof(ResolveReply, ids) + uint32_t(_import_count) * sizeof(uint32_t);
    uint16_t count = 0;
    bool ok = system_reply(result, SystemMethod::Resolve, uint16_t(length), reply) && reply.result >= 0 &&
              uint32_t(result) == length;
    if (ok) {
        memcpy(&count, _platform.link->reply + offsetof(ResolveReply, count), sizeof(count));
        ok = count == _import_count;
    }
    if (!ok) {
        fail_load("API resolve failed");
        return;
    }
    const ModuleLink &link = *_platform.link;
    ResolvedTable table = resolved_table(link, count);
    uint64_t missing = 0;
    const char *missing_name = nullptr;
    if (!_image.check_imports(resolved_id, &table, &missing, &missing_name)) {
        uint32_t text_length = 0;
        char reason[sizeof(_failure)];
        append(reason, sizeof(reason), text_length, "missing API ");
        append(reason, sizeof(reason), text_length, missing_name != nullptr ? missing_name : "?", 63);
        append(reason, sizeof(reason), text_length, " in ");
        append(reason, sizeof(reason), text_length, _name, 31);
        append(reason, sizeof(reason), text_length, " [");
        append_hex(reason, sizeof(reason), text_length, uint32_t(missing >> 32));
        append_hex(reason, sizeof(reason), text_length, uint32_t(missing));
        append(reason, sizeof(reason), text_length, "]");
        fail(reason);
        return;
    }
    // the M4 answers the local ids itself: the M7 must have resolved those hashes as its own
    // catalogue does, or a call would reach the wrong adapter
    for (uint32_t i = 0; i < count; i++) {
        const uint64_t hash = read_u64(table.hashes + i * sizeof(uint64_t));
        const uint32_t id = read_u32(table.ids + i * sizeof(uint32_t));
        uint32_t local_id;
        const bool local_hash = local_import(hash, local_id);
        if ((local_hash && id != local_id) || (!local_hash && AP_Module::binding_is_local(id))) {
            fail_load(CATALOGUE_MISMATCH);
            return;
        }
    }
    // the Stats names of the remote imports, while their names are still staged
    for (uint32_t i = 0; i < count; i++) {
        const uint32_t id = read_u32(table.ids + i * sizeof(uint32_t));
        if (AP_Module::binding_is_local(id)) {
            continue;
        }
        table.calls = 0;
        table.fail_at = i;
        const char *name = nullptr;
        if (!_image.check_imports(resolved_id, &table, nullptr, &name) && name != nullptr) {
            _platform.set_binding_name(_platform.context, id, name);
        }
    }
    _step = Step::Place;
}

void ModuleHost::place()
{
    // thumb: RX in the code window, RW below it. x86-64 (SITL): the PR's contiguous image, RW
    // right after its 4096-aligned RX
    const bool contiguous = _platform.compatibility.machine != 40;
    LoadRequest request {};
    request.file_size = _length;
    request.rx_size = contiguous ? _image.code_size() + _image.rw_memory_size() : _image.code_size();
    request.rx_alignment = contiguous ? 4096U : _image.rx_alignment();
    request.rw_memory_size = contiguous ? 0 : _image.rw_memory_size();
    request.rw_alignment = _image.rw_alignment();
    request.stack_size = uint32_t(_config.stack_size);
    request.heap_size = uint32_t(_config.heap_size);
    if (request.heap_size > MODULE_HEAP_MAX ||
        load_layout(_platform.area_base, _platform.area_size, request, _layout) != LoadFit::Ok) {
        fail_load(NO_ROOM);
        return;
    }
    _rx_base = _layout.window.rx_base;
    _rx_size = request.rx_size;
    _rw_base = contiguous ? _rx_base + _image.code_size() : _layout.rw_base;
    _step = Step::Load;
}

void ModuleHost::load()
{
    ResolvedTable table = resolved_table(*_platform.link, _import_count);
    if (!_image.load_split(pointer(_rx_base), _image.code_size(), pointer(_rw_base), _image.rw_memory_size(),
                           resolved_id, &table)) {
        fail_load("image relocation/publication failed");
        return;
    }
    _step = Step::Checksum;
}

void ModuleHost::publish()
{
    // nothing stale above the code may become executable with it
    const uint32_t window_top = _layout.window.base + _layout.window.size;
    memset(pointer(_rx_base + _rx_size), 0, window_top - (_rx_base + _rx_size));
    if (_platform.publish != nullptr && !_platform.publish(_platform.context, pointer(_rx_base), _image.code_size())) {
        fail_load("image relocation/publication failed");
        return;
    }
    // staging is released: the stack and the heap from the bottom of the area
    ModuleContext &m = _module;
    m.index = 0;
    m.request = _platform.link->request;
    m.reply = _platform.link->reply;
    m.panic_text[0] = 0;
    m.ranges.rx = { uintptr_t(pointer(_rx_base)), _image.code_size() };
    m.ranges.rw = { uintptr_t(pointer(_rw_base)), _image.rw_memory_size() };
    m.ranges.stack = { uintptr_t(pointer(_layout.stack_base)), _layout.stack_top - _layout.stack_base };
    m.ranges.heap = { uintptr_t(pointer(_layout.heap_base)), _layout.heap_end - _layout.heap_base };
    if (!m.heap.init(m.ranges.heap.base, m.ranges.heap.size, _heap_used, _heap_starts,
                     uint32_t(sizeof(_heap_used) / sizeof(_heap_used[0])))) {
        fail_load(NO_ROOM);
        return;
    }
    m.state.store(uint8_t(ModuleStateKind::Loaded));
    post_module_state(ModuleStateKind::Loaded);
    _start_phase = 0;
    _step = Step::Start;
}

// Started is sent before the thread starts: the module's own calls use the link from then on
void ModuleHost::start()
{
    if (_start_phase == 0) {
        _start_phase = 1;
        post_module_state(ModuleStateKind::Started);
        return;
    }
    const uint32_t entry = _rx_base + _image.entry_offset();
    _platform.start(_platform.context, _module, _layout, entry, _rw_base);
    // refused while another module thread still runs
    if (_module.state.load() != uint8_t(ModuleStateKind::Started)) {
        fail("module heap/thread creation failed");
        return;
    }
    _step = Step::Running;
}

}  // namespace ModuleM4

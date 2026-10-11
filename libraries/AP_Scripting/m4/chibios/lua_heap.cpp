/*
  The Lua heap of the M4 image: newlib-nano's malloc over Lua RAM (_sbrk in
  lua_port.cpp). Runs on the unprivileged Lua thread; its statics, if any,
  are in Lua RAM (Makefile LUA_RAM_OBJECTS).
 */
#include <malloc.h>
#include <stdint.h>
#include <stdlib.h>
#include "../m4_port.h"

void *ScriptingM4::port_heap_realloc(void *block, size_t size)
{
    // newlib-nano's realloc keeps the block whenever it is already big enough
    return realloc(block, size);
}

void ScriptingM4::port_heap_free(void *block)
{
    free(block);
}

// newlib-nano (nano-mallocr.c): a chunk is a 4-byte size word, 0 or 4 bytes of padding
// that 8-align the block (recorded as a negative offset in the word before the block),
// then the block; malloc_usable_size() is the chunk less the size word and the padding
uint32_t ScriptingM4::port_heap_block_cost(void *block)
{
    const int32_t offset = static_cast<const int32_t *>(block)[-1];
    const uint32_t padding = offset < 0 ? uint32_t(-offset) : 0;
    return uint32_t(malloc_usable_size(block)) + 4 + padding;
}

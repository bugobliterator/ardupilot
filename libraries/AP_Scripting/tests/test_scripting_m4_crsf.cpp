#include <AP_gtest.h>
#include <string.h>
#include <stdlib.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scripting/AP_ScriptingM4_CRSF.h>
#include <AP_Scripting/AP_ScriptingM4_Remote.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_SCRIPTING_M4_ENABLED && AP_CRSF_SCRIPTING_ENABLED

/*
  AP_CRSF_Telem::ScriptedMenu::add_parameter() grows the parameter array with mem_realloc(),
  which leaves the new entry as malloc() gives it. This one gives it filled with 0xA5, as a
  reused heap block can be, so a parameter AP code leaves uninitialised shows here
 */
void *mem_realloc(void *ptr, size_t old_size, size_t new_size)
{
    if (new_size == 0) {
        free(ptr);
        return nullptr;
    }
    void *new_ptr = malloc(new_size);
    if (new_ptr == nullptr) {
        return nullptr;
    }
    memset(new_ptr, 0xA5, new_size);
    if (ptr != nullptr) {
        memcpy(new_ptr, ptr, old_size < new_size ? old_size : new_size);
        free(ptr);
    }
    return new_ptr;
}

namespace ScriptingM4Remote {

using Menu = AP_CRSF_Telem::ScriptedMenu;
using Parameter = AP_CRSF_Telem::ScriptedParameter;

// CRSF telemetry as AP::crsf_telem() makes it (zeroed by NEW_NOTHROW), cleared and freed after the test
class CRSFMenusTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        telem = NEW_NOTHROW AP_CRSF_Telem();
        ASSERT_NE(telem, nullptr);
    }
    void TearDown() override
    {
        crsf_clear_menus(*telem);
        delete telem;
    }

    // a radio read of a parameter (chunk 0 goes to the script, a later chunk straight to the radio)
    bool radio_reads(uint8_t param_num, uint8_t chunk = 0)
    {
        AP_CRSF_Telem::ParameterSettingsReadFrame read {};
        read.destination = AP_CRSF_Protocol::CRSF_ADDRESS_FLIGHT_CONTROLLER;
        read.origin = 0xEA;
        read.param_num = param_num;
        read.param_chunk = chunk;
        return telem->process_scripted_param_read(&read);
    }
    // a radio write of a parameter: a value of 1
    bool radio_writes(uint8_t param_num)
    {
        AP_CRSF_Telem::ParameterSettingsWriteFrame write {};
        write.destination = AP_CRSF_Protocol::CRSF_ADDRESS_FLIGHT_CONTROLLER;
        write.origin = 0xEA;
        write.param_num = param_num;
        write.payload[0] = 1;
        return telem->process_scripted_param_write(&write, 4);
    }

    AP_CRSF_Telem *telem;
};

static const char SELECTION[] = "\x09Mode\0A;B\0\x00\x00\x01";   // a CRSF text selection, as Lua encodes it

// a sub-menu's link is a parameter of no data in its parent, which the menu's destructor frees at
// the session end. AP_CRSF_Telem::ScriptedMenu::add_menu() makes it with add_parameter(0, nullptr),
// which leaves its data pointer as the heap gives it: the session end frees a garbage pointer
TEST_F(CRSFMenusTest, SubMenuLinksHaveNoData)
{
    Menu *root = crsf_make_menu(*telem, nullptr, "Root");
    ASSERT_NE(root, nullptr);
    ASSERT_NE(crsf_make_parameter(*telem, *root, sizeof(SELECTION), SELECTION), nullptr);
    Menu *sub = crsf_make_menu(*telem, root, "Sub");
    ASSERT_NE(sub, nullptr);
    ASSERT_EQ(root->num_params, 2);
    const Parameter &link = root->params[1];
    EXPECT_EQ(link.id, sub->id);
    EXPECT_EQ(link.parent_id, root->id);
    EXPECT_EQ(link.length, 0);
    EXPECT_EQ(link.data, nullptr);
    EXPECT_EQ(sub->parent_id, root->id);
    EXPECT_EQ(sub->num_params, 0);
    // AP_CRSF_Telem's numbering: the menus follow each other with room for their parameters
    // local copies: EXPECT_EQ would odr-use the class constants
    const uint8_t first_id = AP_CRSF_Telem::SCRIPTED_MENU_START_ID;
    const uint8_t menu_size = AP_CRSF_Telem::MAX_SCRIPTED_MENU_SIZE;
    EXPECT_EQ(root->id, first_id);
    EXPECT_EQ(sub->id, root->id + menu_size + 1);
    EXPECT_EQ(root->params[0].id, root->id + 1);
    EXPECT_STREQ(sub->name, "Sub");
    // the radio sees the link as the folder it is
    EXPECT_EQ(telem->scripted_menus.find_menu(link.id), sub);
}

// a parameter of no data would be stored with a null data pointer that AP_CRSF_Telem reads a byte
// of; more than 255 bytes would be cut to the uint8_t length of AP's add_parameter(). Both are
// refused. Without the length checks both are made
TEST_F(CRSFMenusTest, ParameterLengthsAreChecked)
{
    Menu *root = crsf_make_menu(*telem, nullptr, "Root");
    ASSERT_NE(root, nullptr);
    char data[300];
    memset(data, 'x', sizeof(data));
    EXPECT_EQ(crsf_make_parameter(*telem, *root, 0, data), nullptr);
    EXPECT_EQ(crsf_make_parameter(*telem, *root, 256, data), nullptr);
    EXPECT_EQ(crsf_make_parameter(*telem, *root, 300, data), nullptr);
    EXPECT_EQ(root->num_params, 0);
    const Parameter *p = crsf_make_parameter(*telem, *root, 255, data);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->length, 255);
    EXPECT_EQ(memcmp(p->data, data, 255), 0);
    EXPECT_EQ(root->num_params, 1);
}

static void *no_memory(size_t) { return nullptr; }
static void *no_array(size_t, size_t) { return nullptr; }

// the ScriptedMenu constructor copies the name into an unchecked malloc() and leaves calloc()'s
// result unchecked; ScriptedMenu::add_menu() uses add_parameter()'s result unchecked. Without the
// checks here a failed allocation is written through (a fault of the M7) or a menu is linked
// without its parameters
TEST_F(CRSFMenusTest, FailedAllocationsMakeNothing)
{
    const CRSFAllocators no_name { no_memory, calloc };
    const CRSFAllocators no_params { malloc, no_array };
    EXPECT_EQ(crsf_make_menu(*telem, nullptr, "Root", no_name), nullptr);
    EXPECT_EQ(crsf_make_menu(*telem, nullptr, "Root", no_params), nullptr);
    EXPECT_EQ(telem->scripted_menus.next_menu, nullptr);

    Menu *root = crsf_make_menu(*telem, nullptr, "Root");
    ASSERT_NE(root, nullptr);
    EXPECT_EQ(crsf_make_menu(*telem, root, "Sub", no_name), nullptr);
    EXPECT_EQ(crsf_make_menu(*telem, root, "Sub", no_params), nullptr);
    EXPECT_EQ(root->num_params, 0);            // no link to a menu that was not made
    EXPECT_EQ(root->next_menu, nullptr);

    // a full parent takes no sub-menu: its link would be written past the array
    char data[] = "\x01";
    for (uint8_t i = 0; i < CRSF_PARAMETERS_PER_MENU; i++) {
        ASSERT_NE(crsf_make_parameter(*telem, *root, 1, data), nullptr) << int(i);
    }
    EXPECT_EQ(crsf_make_parameter(*telem, *root, 1, data), nullptr);
    EXPECT_EQ(crsf_make_menu(*telem, root, "Sub"), nullptr);
    EXPECT_EQ(root->num_params, CRSF_PARAMETERS_PER_MENU);
    EXPECT_EQ(root->next_menu, nullptr);
}

// a radio request queued in AP_CRSF_Telem points at its parameter. AP's add_parameter() moves the
// array on every call, so a request queued before a later add points into freed memory, which
// get_menu_event() reads and send_write_response() writes. Here the array never moves
TEST_F(CRSFMenusTest, QueuedRequestsSurviveLaterParameters)
{
    Menu *root = crsf_make_menu(*telem, nullptr, "Root");
    ASSERT_NE(root, nullptr);
    const Parameter *first = crsf_make_parameter(*telem, *root, sizeof(SELECTION), SELECTION);
    ASSERT_NE(first, nullptr);
    const uint8_t first_id = first->id;
    const Parameter *params = root->params;
    ASSERT_TRUE(radio_reads(first_id));
    ASSERT_TRUE(radio_writes(first_id));
    for (uint8_t i = 1; i < CRSF_PARAMETERS_PER_MENU; i++) {
        ASSERT_NE(crsf_make_parameter(*telem, *root, sizeof(SELECTION), SELECTION), nullptr) << int(i);
    }
    EXPECT_EQ(root->params, params);
    EXPECT_EQ(first, &root->params[0]);

    uint8_t param_id = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    EXPECT_EQ(telem->get_menu_event(AP_CRSF_Telem::PARAMETER_READ | AP_CRSF_Telem::PARAMETER_WRITE, param_id, payload),
              AP_CRSF_Telem::PARAMETER_READ);
    EXPECT_EQ(param_id, first_id);
    const char answer[] = "\x09Mode\0A;B\0\x01\x00\x01";
    EXPECT_TRUE(telem->send_write_response(sizeof(answer), answer));
    EXPECT_EQ(root->params[0].length, sizeof(answer));
    EXPECT_EQ(memcmp(root->params[0].data, answer, sizeof(answer)), 0);
    EXPECT_EQ(telem->get_menu_event(AP_CRSF_Telem::PARAMETER_READ | AP_CRSF_Telem::PARAMETER_WRITE, param_id, payload),
              AP_CRSF_Telem::PARAMETER_WRITE);
    EXPECT_EQ(param_id, first_id);
}

// the session end frees the menus. Without the queues cleared first a request of the ended session
// stays queued: the next session's get_menu_event()/peek_menu_event() read its parameter, and
// send_write_response() writes it, in freed memory; the CRSF thread answers it from freed memory
TEST_F(CRSFMenusTest, SessionEndDropsQueuedRequests)
{
    Menu *root = crsf_make_menu(*telem, nullptr, "Root");
    ASSERT_NE(root, nullptr);
    char long_data[120];
    memset(long_data, 'x', sizeof(long_data));
    long_data[0] = 0x09;
    const Parameter *p = crsf_make_parameter(*telem, *root, sizeof(long_data), long_data);
    ASSERT_NE(p, nullptr);
    ASSERT_TRUE(radio_reads(p->id));        // inbound, then outbound once the script takes it
    ASSERT_TRUE(radio_writes(p->id));       // inbound
    ASSERT_TRUE(radio_reads(p->id, 1));     // a later chunk: ready for the radio
    uint8_t param_id = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    ASSERT_EQ(telem->get_menu_event(AP_CRSF_Telem::PARAMETER_READ, param_id, payload), AP_CRSF_Telem::PARAMETER_READ);
    EXPECT_EQ(telem->inbound_params.available(), 1U);
    EXPECT_EQ(telem->outbound_params.available(), 1U);
    EXPECT_EQ(telem->ready_params.available(), 1U);

    crsf_clear_menus(*telem);
    EXPECT_EQ(telem->scripted_menus.next_menu, nullptr);
    EXPECT_EQ(telem->inbound_params.available(), 0U);
    EXPECT_EQ(telem->outbound_params.available(), 0U);
    EXPECT_EQ(telem->ready_params.available(), 0U);

    // the next session has none of them
    ASSERT_NE(crsf_make_menu(*telem, nullptr, "Root"), nullptr);
    uint8_t events = 0;
    EXPECT_EQ(telem->peek_menu_event(param_id, payload, events), 0);
    EXPECT_EQ(telem->get_menu_event(AP_CRSF_Telem::PARAMETER_READ | AP_CRSF_Telem::PARAMETER_WRITE, param_id, payload), 0);
    EXPECT_FALSE(telem->send_write_response(1, "x"));
}

// an M4 call waits at most CRSF_LOCK_TIMEOUT_MS for scr_sem. Without the bound the call waits as
// long as another thread holds it (here 300 ms) and then makes the menu
TEST_F(CRSFMenusTest, MenuCallsWaitABoundedTimeForTheLock)
{
    std::atomic<bool> held { false };
    std::thread holder([&]() {
        WITH_SEMAPHORE(telem->get_semaphore());
        held = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    });
    while (!held) {
        std::this_thread::yield();
    }
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(crsf_make_menu(*telem, nullptr, "Root"), nullptr);
    const auto waited = std::chrono::steady_clock::now() - start;
    EXPECT_LT(waited, std::chrono::milliseconds(200));
    holder.join();
    EXPECT_EQ(telem->scripted_menus.next_menu, nullptr);
    EXPECT_NE(crsf_make_menu(*telem, nullptr, "Root"), nullptr);
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED && AP_CRSF_SCRIPTING_ENABLED

AP_GTEST_MAIN()

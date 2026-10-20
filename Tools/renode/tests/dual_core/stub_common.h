#pragma once
#include <cstddef>
#include <cstdint>

namespace DualCoreStub {

constexpr uintptr_t kWindowBase = 0x30040000;   // SRAM3: the Lua shared window (spec §5)
constexpr uint32_t kWindowSize = 32 * 1024;
constexpr uintptr_t kMailboxBase = 0x38000000;  // SRAM4: results read by test_dual_core.py
constexpr uint8_t kDoorbellGroup = 14;          // HSEM 28 (M7 TX) and 29 (M4 TX)
constexpr uint32_t kProtocolVersion = 1;
constexpr uint32_t kContractHash = 0x53545542;  // "STUB"
constexpr uint32_t kRoundTrips = 100;
constexpr size_t kFrameWords = 4;
constexpr uint32_t kHelloMagic = 0x48454C4F;    // "HELO"
constexpr uint32_t kPingMagic = 0x50494E47;     // "PING"
constexpr uint32_t kPongMagic = 0x504F4E47;     // "PONG"
constexpr uint32_t kDoneMagic = 0x444F4E45;     // "DONE"

enum Stage : uint32_t {
    StageBoot = 1,
    StageWindowReady,
    StageM4Released,
    StagePeerCompatible,
    StageRoundTrips,
    StageFinished,
};

enum Error : uint32_t {
    ErrorChannelInit = 1,
    ErrorHello,
    ErrorPeerHeader,
    ErrorSend,
    ErrorPong,
    ErrorPing,
};

struct Mailbox {
    volatile uint32_t m7Stage;
    volatile uint32_t m7PeerStatus;     // PeerHeader::Status of the M4 half, seen by the M7
    volatile uint32_t roundTrips;
    volatile uint32_t m7DoorbellIrqs;
    volatile uint32_t m7Error;
    volatile uint32_t m4Stage;
    volatile uint32_t m4PeerStatus;     // PeerHeader::Status of the M7 half, seen by the M4
    volatile uint32_t pingsAnswered;
    volatile uint32_t m4DoorbellIrqs;
    volatile uint32_t m4CpuId;
    volatile uint32_t m4Error;
    volatile uint32_t done;
};
static_assert(offsetof(Mailbox, m4Stage) == 0x14 && offsetof(Mailbox, done) == 0x2C,
              "test_dual_core.py reads this layout (MAILBOX_FIELDS)");

inline Mailbox *mailbox()
{
    return reinterpret_cast<Mailbox *>(kMailboxBase);
}

}

// Boot-handshake stub, Cortex-M7 side: owns the shared window, releases the M4,
// then runs kRoundTrips PING/PONG exchanges over the CubeFramework Channel.
#include "stub_common.h"
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/pal/HSEM.h>
#include <cerrno>
#include <cstring>

using namespace CubePilotFW;
using namespace DualCoreStub;

static Channel channel;

extern "C" void HSEM_IRQHandler(void)
{
    HwSemaphore::handleIrq();
    mailbox()->m7DoorbellIrqs = mailbox()->m7DoorbellIrqs + 1;
}

[[noreturn]] static void fail(uint32_t code)
{
    mailbox()->m7Error = code;
    for (;;) {
        __WFI();
    }
}

// The unbounded sleep below is safe only here: each TX semaphore has a single taker and
// the peer is always live. Do not copy it into the M4 Boot & Supervisor code, which must
// follow Channel's bounded-sleep consumer contract (a doorbell can be deferred while the
// HSEM is busy).
static void sleepUntilDoorbell()
{
    __disable_irq();
    if (channel.prepareSleep()) {
        __WFI();  // a pending HSEM interrupt ends WFI even with PRIMASK set
    }
    __enable_irq();  // the doorbell ISR runs here
    channel.wake();
}

static void receiveFrame(uint32_t (&frame)[kFrameWords], uint32_t failCode)
{
    uint16_t len = 0;
    int ret;
    while ((ret = channel.receive(frame, sizeof frame, &len)) == -EAGAIN) {
        sleepUntilDoorbell();
    }
    if (ret != 0 || len != sizeof frame) {
        fail(failCode);
    }
}

int main()
{
    Mailbox *box = mailbox();
    memset(reinterpret_cast<void *>(kMailboxBase), 0, sizeof(Mailbox));
    box->m7Stage = StageBoot;
    uint8_t *window = reinterpret_cast<uint8_t *>(kWindowBase);
    memset(window, 0, kWindowSize);  // cold boot: the M7 owns the window layout
    HwSemaphore::enableClock();  // the HSEM clock must be on before init() touches HSEM registers
    HwSemaphore::init();         // clears stale enables and flags before the IRQ line is enabled
    if (channel.init(window, kWindowSize, Channel::Core::M7, kDoorbellGroup) != 0) {
        fail(ErrorChannelInit);
    }
    PeerHeader::publish(window, PeerHeader::Core::M7, kProtocolVersion, kContractHash, 0);
    HwSemaphore::enableRxIrq(kDoorbellGroup, true);
    HwSemaphore::enableNvic(8);
    box->m7Stage = StageWindowReady;

    RCC->GCR |= RCC_GCR_BOOT_C2;
    box->m7Stage = StageM4Released;

    uint32_t frame[kFrameWords];
    receiveFrame(frame, ErrorHello);
    if (frame[0] != kHelloMagic) {
        fail(ErrorHello);
    }
    PeerHeader::View view;
    const PeerHeader::Status status =
        PeerHeader::readPeer(window, PeerHeader::Core::M7, kProtocolVersion, kContractHash, &view);
    box->m7PeerStatus = static_cast<uint32_t>(status);
    if (status != PeerHeader::Status::Compatible) {
        fail(ErrorPeerHeader);
    }
    box->m7Stage = StagePeerCompatible;

    box->m7Stage = StageRoundTrips;
    for (uint32_t seq = 1; seq <= kRoundTrips; ++seq) {
        const uint32_t ping[kFrameWords] = {kPingMagic, seq, ~seq, 0};
        if (channel.send(ping, sizeof ping) != 0) {
            fail(ErrorSend);
        }
        receiveFrame(frame, ErrorPong);
        if (frame[0] != kPongMagic || frame[1] != seq || frame[2] != ~seq) {
            fail(ErrorPong);
        }
        box->roundTrips = seq;
        PeerHeader::beat(window, PeerHeader::Core::M7);
    }
    box->m7Stage = StageFinished;
    box->done = kDoneMagic;
    for (;;) {
        __WFI();
    }
}

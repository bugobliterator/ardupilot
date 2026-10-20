// Boot-handshake stub, Cortex-M4 side: joins the window the M7 prepared,
// announces itself, then answers every PING with a PONG.
#include "stub_common.h"
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/pal/HSEM.h>
#include <cerrno>

using namespace CubePilotFW;
using namespace DualCoreStub;

static Channel channel;

extern "C" void HSEM_IRQHandler(void)
{
    HwSemaphore::handleIrq();
    mailbox()->m4DoorbellIrqs = mailbox()->m4DoorbellIrqs + 1;
}

[[noreturn]] static void fail(uint32_t code)
{
    mailbox()->m4Error = code;
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
        __WFI();
    }
    __enable_irq();
    channel.wake();
}

int main()
{
    Mailbox *box = mailbox();
    box->m4CpuId = SCB->CPUID;
    box->m4Stage = StageBoot;
    uint8_t *window = reinterpret_cast<uint8_t *>(kWindowBase);
    HwSemaphore::init();  // the M7 enabled the HSEM clock; enableClock() is a no-op on the M4
    if (channel.init(window, kWindowSize, Channel::Core::M4, kDoorbellGroup) != 0) {
        fail(ErrorChannelInit);
    }
    channel.resetAfterRestart();
    PeerHeader::publish(window, PeerHeader::Core::M4, kProtocolVersion, kContractHash, 0);
    PeerHeader::View view;
    const PeerHeader::Status status =
        PeerHeader::readPeer(window, PeerHeader::Core::M4, kProtocolVersion, kContractHash, &view);
    box->m4PeerStatus = static_cast<uint32_t>(status);
    if (status != PeerHeader::Status::Compatible) {
        fail(ErrorPeerHeader);
    }
    HwSemaphore::enableRxIrq(kDoorbellGroup, true);
    HwSemaphore::enableNvic(8);
    box->m4Stage = StagePeerCompatible;

    const uint32_t hello[kFrameWords] = {kHelloMagic, SCB->CPUID, 0, 0};
    if (channel.send(hello, sizeof hello) != 0) {
        fail(ErrorHello);
    }
    box->m4Stage = StageRoundTrips;
    for (;;) {
        uint32_t frame[kFrameWords];
        uint16_t len = 0;
        const int ret = channel.receive(frame, sizeof frame, &len);
        if (ret == -EAGAIN) {
            sleepUntilDoorbell();
            continue;
        }
        if (ret != 0 || len != sizeof frame || frame[0] != kPingMagic) {
            fail(ErrorPing);
        }
        frame[0] = kPongMagic;
        if (channel.send(frame, sizeof frame) != 0) {
            fail(ErrorSend);
        }
        box->pingsAnswered = box->pingsAnswered + 1;
        PeerHeader::beat(window, PeerHeader::Core::M4);
    }
}

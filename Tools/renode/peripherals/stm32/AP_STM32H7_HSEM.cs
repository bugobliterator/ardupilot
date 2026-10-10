// STM32H7 hardware semaphore (RM0399 HSEM) for single- and dual-core H7.
// Renode's STM32H7_HardwareSemaphore treats a 2-step write with LOCK=1 as an
// unlock and has no interrupts. This model:
//  - takes COREID from the bus initiator (Cpu1/Cortex-M7 = 3, Cpu2/Cortex-M4 = 1)
//    and ignores 2-step writes whose COREID field is not the initiator's
//  - frees only on a write by the owner with matching COREID and PROCID
//  - on every free sets the semaphore's bit in C1ISR and C2ISR; CnMISR = CnISR &
//    CnIER drives C1IRQ (HSEM1, M7 IRQ 125) and C2IRQ (HSEM2, M4 IRQ 126)
//  - implements HSEM_CR: free every semaphore of a COREID when KEY matches KEYR
// Lock, take and free paths act only for a CPU executing guest code on its own
// thread; monitor and debugger accesses (also those issued in a CPU context)
// leave semaphores unchanged. Monitor writes to CR, ICR, IER and KEYR take effect.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32H7_HSEM : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_HSEM(IMachine machine, IPeripheral cpu1 = null, IPeripheral cpu2 = null)
        {
            this.machine = machine;
            Cpu1 = cpu1;
            Cpu2 = cpu2;
            C1IRQ = new GPIO();
            C2IRQ = new GPIO();
            Reset();
        }

        public GPIO C1IRQ { get; }
        public GPIO C2IRQ { get; }
        public IPeripheral Cpu1 { get; set; }
        public IPeripheral Cpu2 { get; set; }
        public long Size => 0x400;

        public void Reset()
        {
            lock(sync)
            {
                Array.Clear(owner, 0, SemaphoreCount);
                Array.Clear(processId, 0, SemaphoreCount);
                Array.Clear(interruptEnable, 0, interruptEnable.Length);
                Array.Clear(interruptStatus, 0, interruptStatus.Length);
                clearKey = 0;
                UpdateInterrupts();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            lock(sync)
            {
                if(offset >= SemaphoreOffset && offset < ReadLockOffset)
                {
                    return SemaphoreValue((int)(offset / 4));
                }
                if(offset >= ReadLockOffset && offset < C1IEROffset)
                {
                    var index = (int)((offset - ReadLockOffset) / 4);
                    var core = InitiatorCoreId();
                    if(core != 0 && owner[index] == 0)
                    {
                        owner[index] = core;
                        processId[index] = 0;
                    }
                    return SemaphoreValue(index);
                }
                switch(offset)
                {
                case C1IEROffset: return interruptEnable[0];
                case C1ISROffset: return interruptStatus[0];
                case C1MISROffset: return interruptStatus[0] & interruptEnable[0];
                case C2IEROffset: return interruptEnable[1];
                case C2ISROffset: return interruptStatus[1];
                case C2MISROffset: return interruptStatus[1] & interruptEnable[1];
                case KEYROffset: return clearKey << 16;
                default: return 0;  // ICR and CR are write-only
                }
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(sync)
            {
                if(offset >= SemaphoreOffset && offset < ReadLockOffset)
                {
                    WriteSemaphore((int)(offset / 4), value);
                    return;
                }
                switch(offset)
                {
                case C1IEROffset: interruptEnable[0] = value; break;
                case C1ICROffset: interruptStatus[0] &= ~value; break;
                case C2IEROffset: interruptEnable[1] = value; break;
                case C2ICROffset: interruptStatus[1] &= ~value; break;
                case CROffset: ClearByCore(value); break;
                case KEYROffset: clearKey = value >> 16; break;
                default:
                    this.Log(LogLevel.Warning, "Write 0x{0:X} to read-only or reserved offset 0x{1:X}", value, offset);
                    return;
                }
                UpdateInterrupts();
            }
        }

        private void WriteSemaphore(int index, uint value)
        {
            var core = InitiatorCoreId();
            var writtenCore = (value >> CoreIdShift) & 0xFF;
            var writtenProcess = value & 0xFF;
            if(core == 0 || writtenCore != core)
            {
                this.Log(LogLevel.Debug, "Semaphore {0}: write 0x{1:X} from COREID {2} ignored", index, value, core);
                return;
            }
            if((value & LockBit) != 0)
            {
                if(owner[index] == 0)
                {
                    owner[index] = core;
                    processId[index] = writtenProcess;
                }
                return;
            }
            if(owner[index] == core && processId[index] == writtenProcess)
            {
                Free(index);
                UpdateInterrupts();
            }
        }

        private void ClearByCore(uint value)
        {
            if((value >> 16) != clearKey)
            {
                this.Log(LogLevel.Debug, "HSEM_CR key 0x{0:X} does not match KEYR", value >> 16);
                return;
            }
            var core = (value >> CoreIdShift) & 0xFF;
            for(var index = 0; index < SemaphoreCount; index++)
            {
                if(core != 0 && owner[index] == core)
                {
                    Free(index);
                }
            }
        }

        private void Free(int index)
        {
            owner[index] = 0;
            processId[index] = 0;
            interruptStatus[0] |= 1u << index;
            interruptStatus[1] |= 1u << index;
        }

        private uint SemaphoreValue(int index)
        {
            return owner[index] == 0 ? 0u : LockBit | (owner[index] << CoreIdShift) | processId[index];
        }

        private uint InitiatorCoreId()
        {
            if(!machine.SystemBus.TryGetCurrentCPU(out var cpu) || !cpu.OnPossessedThread)
            {
                return 0;
            }
            if(ReferenceEquals(cpu, Cpu1))
            {
                return CoreIdCpu1;
            }
            if(ReferenceEquals(cpu, Cpu2))
            {
                return CoreIdCpu2;
            }
            return 0;
        }

        private void UpdateInterrupts()
        {
            C1IRQ.Set((interruptStatus[0] & interruptEnable[0]) != 0);
            C2IRQ.Set((interruptStatus[1] & interruptEnable[1]) != 0);
        }

        private readonly IMachine machine;
        private readonly object sync = new object();
        private readonly uint[] owner = new uint[SemaphoreCount];
        private readonly uint[] processId = new uint[SemaphoreCount];
        private readonly uint[] interruptEnable = new uint[2];
        private readonly uint[] interruptStatus = new uint[2];
        private uint clearKey;

        private const int SemaphoreCount = 32;
        private const int CoreIdShift = 8;
        private const uint LockBit = 1u << 31;
        private const uint CoreIdCpu1 = 3;
        private const uint CoreIdCpu2 = 1;
        private const long SemaphoreOffset = 0x000;
        private const long ReadLockOffset = 0x080;
        private const long C1IEROffset = 0x100;
        private const long C1ICROffset = 0x104;
        private const long C1ISROffset = 0x108;
        private const long C1MISROffset = 0x10C;
        private const long C2IEROffset = 0x110;
        private const long C2ICROffset = 0x114;
        private const long C2ISROffset = 0x118;
        private const long C2MISROffset = 0x11C;
        private const long CROffset = 0x140;
        private const long KEYROffset = 0x144;
    }
}

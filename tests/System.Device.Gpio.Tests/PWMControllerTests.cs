// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.

using System.Device.Pwm;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;
using Xunit;

namespace System.Device.Gpio.Tests
{
    public unsafe class PWMControllerTests
    {
        private const int PwmChip = 0;
        private const int PwmChannel = 0;

        [DllImport("libc", SetLastError = true)]
        internal static extern int open([MarshalAs(UnmanagedType.LPStr)] string pathname, FileOpenFlags flags);

        [DllImport("libc", SetLastError = true)]
        internal static extern IntPtr mmap(IntPtr addr, int length, MemoryMappedProtections prot, MemoryMappedFlags flags, int fd, int offset);

        [Flags]
        internal enum MemoryMappedProtections
        {
            PROT_NONE = 0x0,
            PROT_READ = 0x1,
            PROT_WRITE = 0x2,
            PROT_EXEC = 0x4
        }

        [Flags]
        internal enum MemoryMappedFlags
        {
            MAP_SHARED = 0x01,
            MAP_PRIVATE = 0x02,
            MAP_FIXED = 0x10
        }

        internal enum FileOpenFlags
        {
            O_RDONLY = 0x00,
            O_RDWR = 0x02,
            O_NONBLOCK = 0x800,
            O_SYNC = 0x101000
        }

        [DllImport("libc", SetLastError = true)]
        internal static extern int close(int fd);

        [Fact]
        public void CanStartStopPWM()
        {
            var fd = open("/dev/gpiomem", FileOpenFlags.O_RDWR | FileOpenFlags.O_SYNC);
            IntPtr mapPointer = mmap(IntPtr.Zero, Environment.SystemPageSize, MemoryMappedProtections.PROT_READ | MemoryMappedProtections.PROT_WRITE, MemoryMappedFlags.MAP_SHARED, fd, 0);
            _registers = (RegisterView*)mapPointer;
            close(fd);

            using (GpioController gpController = new GpioController())
            using (PwmController controller = new PwmController())
            {
                gpController.OpenPin(18, PinMode.Output);
                SetPinMode(18, 0x02);
                controller.OpenChannel(PwmChip, PwmChannel);
                controller.StartWriting(PwmChip, PwmChannel, 800, 50);
                Thread.Sleep(TimeSpan.FromSeconds(2));
                controller.ChangeDutyCycle(PwmChip, PwmChannel, 99);
                Thread.Sleep(TimeSpan.FromSeconds(2));
                controller.StopWriting(PwmChip, PwmChannel);
            }
        }

        private RegisterView* _registers;

        private void SetPinMode(int gpioPinNumber, uint mode)
        {
            //SetBits(RegisterViewPointer->GPFSEL, pin, 10, 3, mode);

            int index = gpioPinNumber / 10;
            int shift = (gpioPinNumber % 10) * 3;
            uint* registerPointer = &_registers->GPFSEL[index];

            //Console.WriteLine($"{nameof(RegisterView.GPFSEL)} register address = {(long)registerPointer:X16}");

            uint register = *registerPointer;

            //Console.WriteLine($"{nameof(RegisterView.GPFSEL)} original register value = {register:X8}");

            register &= ~(0b111U << shift);
            register |= mode << shift;

            //Console.WriteLine($"{nameof(RegisterView.GPFSEL)} new register value = {register:X8}");

            *registerPointer = register;
        }

        [StructLayout(LayoutKind.Explicit)]
        internal struct RegisterView
        {
            ///<summary>GPIO Function Select, 6x32 bits, R/W</summary>
            [FieldOffset(0x00)]
            public fixed uint GPFSEL[6];

            ///<summary>GPIO Pin Output Set, 2x32 bits, W</summary>
            [FieldOffset(0x1C)]
            public fixed uint GPSET[2];

            ///<summary>GPIO Pin Output Clear, 2x32 bits, W</summary>
            [FieldOffset(0x28)]
            public fixed uint GPCLR[2];

            ///<summary>GPIO Pin Level, 2x32 bits, R</summary>
            [FieldOffset(0x34)]
            public fixed uint GPLEV[2];

            ///<summary>GPIO Pin Event Detect Status, 2x32 bits, R/W</summary>
            [FieldOffset(0x40)]
            public fixed uint GPEDS[2];

            ///<summary>GPIO Pin Rising Edge Detect Enable, 2x32 bits, R/W</summary>
            [FieldOffset(0x4C)]
            public fixed uint GPREN[2];

            ///<summary>GPIO Pin Falling Edge Detect Enable, 2x32 bits, R/W</summary>
            [FieldOffset(0x58)]
            public fixed uint GPFEN[2];

            ///<summary>GPIO Pin High Detect Enable, 2x32 bits, R/W</summary>
            [FieldOffset(0x64)]
            public fixed uint GPHEN[2];

            ///<summary>GPIO Pin Low Detect Enable, 2x32 bits, R/W</summary>
            [FieldOffset(0x70)]
            public fixed uint GPLEN[2];

            ///<summary>GPIO Pin Async. Rising Edge Detect, 2x32 bits, R/W</summary>
            [FieldOffset(0x7C)]
            public fixed uint GPAREN[2];

            ///<summary>GPIO Pin Async. Falling Edge Detect, 2x32 bits, R/W</summary>
            [FieldOffset(0x88)]
            public fixed uint GPAFEN[2];

            ///<summary>GPIO Pin Pull-up/down Enable, 32 bits, R/W</summary>
            [FieldOffset(0x94)]
            public uint GPPUD;

            ///<summary>GPIO Pin Pull-up/down Enable Clock, 2x32 bits, R/W</summary>
            [FieldOffset(0x98)]
            public fixed uint GPPUDCLK[2];
        }
    }
}

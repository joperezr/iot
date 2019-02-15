﻿// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.

using System.Device.I2c;
using System.Device.I2c.Drivers;
using System.Diagnostics;
using System.Threading;
using Iot.Device.Mcp23xxx;

namespace Iot.Device.CharacterLcd.Samples
{
    class Program
    {
        /// <summary>
        /// This program will print `Hello World`
        /// </summary>
        /// <param name="args">Should be empty</param>
        static void Main(string[] args)
        {
            // Sets up a 16x2 character LCD with a hardwired or no backlight.
            using (Lcd2004 lcd = new Lcd2004(registerSelect: 22, enable: 17, data: new int[] { 25, 24, 23, 18 }))
            {
                lcd.Clear();
                lcd.Write("Hello World");
                lcd.SetCursorPosition(0, 1);
                lcd.Write(".NET Core!");
                Stopwatch sw = new Stopwatch();
                sw.Start();
                while (sw.ElapsedMilliseconds < 20_000)
                {
                    lcd.SetCursorPosition(0,2);
                    lcd.Write($"{(sw.ElapsedMilliseconds/1000.0):0.000} seconds");
                    Thread.Sleep(10);
                }
                lcd.Clear();
                lcd.Home();
                lcd.Write("Goodbye!");
            }
        }

        /// <summary>
        /// This method will use an mcp gpio extender to connect to the LCM display.
        /// This has been tested on the CrowPi lcd display.
        /// </summary>
        static void UsingMcp()
        {
            UnixI2cDevice i2CDevice = new UnixI2cDevice(new I2cConnectionSettings(1, 0x21));
            Mcp23008 mcpDevice = new Mcp23008(i2CDevice);
            int[] dataPins = { 3, 4, 5, 6 };
            int registerSelectPin = 1;
            int enablePin = 2;
            int backlight = 7;
            using (mcpDevice)
            using (Lcd1602 lcd = new Lcd1602(registerSelectPin, enablePin, dataPins, backlight, controller: mcpDevice))
            {
                lcd.Clear();

                lcd.Write("Hello World");
                lcd.SetCursorPosition(0, 1);
                lcd.Write(".NET Core");
            }
        }
    }
}

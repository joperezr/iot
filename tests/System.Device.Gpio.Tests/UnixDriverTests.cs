﻿// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.

using System.Device.Gpio.Drivers;

namespace System.Device.Gpio.Tests
{
    public class UnixDriverTests : GpioControllerTestBase
    {
        protected override GpioDriver GetTestDriver() => new UnixDriver();

        protected override PinNumberingScheme GetTestNumberingScheme() => PinNumberingScheme.Logical;
    }
}

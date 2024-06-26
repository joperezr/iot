﻿// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Iot.Device.Arduino;
using Iot.Device.Board;
using Iot.Device.Common;
using Microsoft.Extensions.Logging;

namespace ArduinoCsCompiler
{
    /// <summary>
    /// This class helps creating the interface .h files in the runtime. They need to be in sync with the version used by the compiler.
    /// </summary>
    public class WriteRuntimeCoreData
    {
        private readonly ILogger _logger;
        private string _targetPath;
        private string _targetRootPath;

        /// <summary>
        /// Write the interface header files required by the runtime to the standard path
        /// </summary>
        public WriteRuntimeCoreData()
        : this(null)
        {
        }

        /// <summary>
        /// Write the interface header files required by the runtime
        /// </summary>
        /// <param name="toPath">Destination path. If null, defaults to the value returned by <see cref="GetRuntimePath"/>.</param>
        public WriteRuntimeCoreData(string? toPath)
        {
            _logger = this.GetCurrentClassLogger();
            if (toPath == null)
            {
                _targetRootPath = GetRuntimePath();
            }
            else
            {
                _targetRootPath = toPath;
            }

            _targetPath = Path.Combine(_targetRootPath, "interface");
        }

        public string TargetPath => _targetPath;

        public string TargetRootPath => _targetRootPath;

        /// <summary>
        /// Writes the data.
        /// </summary>
        public void Write()
        {
            if (!Directory.Exists(_targetRootPath))
            {
                _logger.LogWarning($"Warning: {_targetRootPath} does not exist. Please ensure it is correct and make sure the runtime is checked out correctly");
            }

            Directory.CreateDirectory(_targetPath);
            WriteBreakpointTypes();
            WriteNativeMethodDefinitions();
            WriteDebuggerCommands();
            WriteExceptionClauseTypes();
            WriteKnownTypeTokens();
            WriteMethodFlags();
            WritePinUsage();
            WriteRuntimeState();
            WriteSystemExceptions();
            WriteExecutorCommands();
            WriteVariableKind();
        }

        /// <summary>
        /// Returns the path where the runtime sources are
        /// </summary>
        /// <returns>The default installation location for the runtime, within the arduino libraries (i.e. c:\users\Username\Documents\Arduino\ExtendedConfigurableFirmata)</returns>
        private string GetRuntimePath()
        {
            string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
            return Path.Combine(path, @"Arduino\ExtendedConfigurableFirmata");
        }

        private void WriteNativeMethodDefinitions()
        {
            // WriteEnumHeaderFile<NativeMethod>();
            // Collect all methods that have an ArduinoImplementation attribute attached to them.
            // Must scan all functions, including internals.
            Assembly[] typesWhereToLook = new Assembly[]
            {
                Assembly.GetAssembly(typeof(MicroCompiler))!,
                Assembly.GetAssembly(typeof(ArduinoBoard))!
            };

            string[] specials = new string[]
            {
                "ByReferenceCtor",
                "ByReferenceValue",
            };

            Dictionary<string, int> entries = new();

            foreach (var s in specials)
            {
                entries.Add(s, ArduinoImplementationAttribute.GetStaticHashCode(s));
            }

            // The loop below will throw on duplicate entries. This is expected.
            foreach (var a in typesWhereToLook)
            {
                foreach (var type in a.GetTypes())
                {
                    foreach (var method in type.GetMethods(BindingFlags.DeclaredOnly | BindingFlags.Instance | BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public))
                    {
                        var attribs = method.GetCustomAttributes(typeof(ArduinoImplementationAttribute)).Cast<ArduinoImplementationAttribute>();
                        var attrib = attribs.FirstOrDefault();
                        if (attrib != null && attrib.MethodNumber != 0)
                        {
                            TryAddEntry(entries, attrib);
                        }
                    }

                    foreach (var method in type.GetConstructors(BindingFlags.DeclaredOnly | BindingFlags.Instance | BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public))
                    {
                        var attribs = method.GetCustomAttributes(typeof(ArduinoImplementationAttribute)).Cast<ArduinoImplementationAttribute>();
                        var attrib = attribs.FirstOrDefault();
                        if (attrib != null && attrib.MethodNumber != 0)
                        {
                            TryAddEntry(entries, attrib);
                        }
                    }
                }
            }

            IEnumerable<int> duplicates = entries.GroupBy(x => x.Value)
                .Where(g => g.Count() > 1)
                .Select(x => x.Key).ToList();
            if (duplicates.Any())
            {
                throw new InvalidOperationException($"Duplicate method keys found: {duplicates.First()}");
            }

            var list = entries.OrderBy(x => x.Key).Select(y => (y.Key, y.Value));
            var reverseLookupList = entries.OrderBy(x => x.Value).Select(y => (y.Value, y.Key));
            WriteNativeMethodList(list, reverseLookupList);
        }

        private void TryAddEntry(Dictionary<string, int> entries, ArduinoImplementationAttribute attrib)
        {
            if (!entries.ContainsKey(attrib.Name))
            {
                entries.Add(attrib.Name, attrib.MethodNumber);
            }
            else if (entries[attrib.Name] == attrib.MethodNumber)
            {
                // Nothing to do
            }
            else
            {
                throw new InvalidOperationException($"Method {attrib.Name} was already declared with a different hash code");
            }
        }

        private void WriteNativeMethodList(IEnumerable<(string Key, int Value)> entries, IEnumerable<(int Value, string Key)> reverseLookupList)
        {
            string name = "NativeMethod";
            string header = FormattableString.Invariant($@"
#pragma once

// This file is autogenerated. Any edits will be lost after running the compiler tests to update the references
// Native method numbers, ordered by method name
enum class {name}
{{
    None = 0,
");
            string outputFile = Path.Combine(_targetPath, name + ".h");
            TextWriter w = new StreamWriter(outputFile, false, Encoding.ASCII);
            w.Write(header);
            foreach (var e in entries)
            {
                w.WriteLine(FormattableString.Invariant($"    {e.Key} = {e.Value},"));
            }

            w.WriteLine("};"); // Tail

            w.WriteLine("/* Reverse lookup list (ordered by value)");
            foreach (var e in reverseLookupList)
            {
                w.WriteLine($"{e.Value} (0x{e.Value:X}) -> {e.Key}");
            }

            w.WriteLine("*/");
            w.Close();
        }

        private void WriteSystemExceptions()
        {
            WriteEnumHeaderFile<SystemException>();
        }

        private void WriteMethodFlags()
        {
            WriteEnumHeaderFile<MethodFlags>();
        }

        private void WriteKnownTypeTokens()
        {
            WriteEnumHeaderFile<KnownTypeTokens>();
        }

        private void WriteExceptionClauseTypes()
        {
            WriteEnumHeaderFile<ExceptionHandlingClauseOptions>();
        }

        private void WriteRuntimeState()
        {
            WriteEnumHeaderFile<RuntimeState>();
        }

        private void WriteDebuggerCommands()
        {
            WriteEnumHeaderFile<DebuggerCommand>();
        }

        private void WriteBreakpointTypes()
        {
            WriteEnumHeaderFile<BreakpointType>();
        }

        private void WritePinUsage()
        {
            WriteEnumHeaderFile<PinUsage>();
        }

        private void WriteExecutorCommands()
        {
            string name = nameof(ExecutorCommand);
            string header = FormattableString.Invariant($@"
#pragma once

// This file is autogenerated. Any edits will be lost after running the compiler tests to update the references
enum class {name} : byte
{{
");
            string outputFile = Path.Combine(_targetPath, name + ".h");
            TextWriter w = new StreamWriter(outputFile, false, Encoding.ASCII);
            w.Write(header);
            foreach (var e in Enum.GetValues(typeof(ExecutorCommand)))
            {
                w.WriteLine(FormattableString.Invariant($"    {e.ToString()} = {(byte)e},"));
            }

            w.WriteLine("};"); // Tail
            w.Close();
        }

        private void WriteVariableKind()
        {
            WriteEnumHeaderFile<VariableKind>();
        }

        private void WriteEnumHeaderFile<T>()
            where T : struct, Enum
        {
            string name = typeof(T).Name;
            string size = string.Empty;
            if (Enum.GetUnderlyingType(typeof(T)) == typeof(byte))
            {
                size = " : byte";
            }

            string header = FormattableString.Invariant($@"
#pragma once

// This file is autogenerated. Any edits will be lost after running the compiler tests to update the references
enum class {name}{size}
{{
");
            string outputFile = Path.Combine(_targetPath, name + ".h");
            // Must use ascii encoding, because GCC fails to recognize the UTF-8-BOM header
            // sometimes. Not sure why it works sometimes only.
            TextWriter w = new StreamWriter(outputFile, false, Encoding.ASCII);
            w.Write(header);
            foreach (var e in Enum.GetValues<T>())
            {
                w.WriteLine(FormattableString.Invariant($"    {e.ToString()} = {GetIntValueFromEnum(e)},"));
            }

            w.WriteLine("};"); // Tail
            w.Close();
        }

        private int GetIntValueFromEnum<T>(T value)
            where T : Enum
        {
            return Convert.ToInt32(value);
        }
    }
}

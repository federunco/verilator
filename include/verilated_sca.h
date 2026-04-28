// -*- mode: C++; c-file-style: "cc-mode" -*-
//=============================================================================
//
// Copyright 2026 by Federico Runco. This program is free software; you
// can redistribute it and/or modify it under the terms of either the GNU
// Lesser General Public License Version 3 or the Perl Artistic License
// Version 2.0.
// SPDX-License-Identifier: LGPL-3.0-only OR Artistic-2.0
//
//=============================================================================
///
/// \file
/// \brief Verilated HW/HD tracing for Side Channel Analysis
///
/// This header provides VerilatedSca, a runtime class that records
/// per-timestep Hamming Weight or Hamming Distance of all signals
/// within a scoped module hierarchy. Output is written to sequentially
/// numbered CSV files, gated by a trigger signal.
///
//=============================================================================

#ifndef VERILATOR_VERILATED_SCA_H_
#define VERILATOR_VERILATED_SCA_H_

#include "verilatedos.h"

#include "verilated.h"
#include "verilated_syms.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

//=============================================================================
// VerilatedSca - Side Channel Analysis tracer

class VerilatedSca final {
    // TYPES
    struct SignalInfo {
        void* datap = nullptr;  // Direct pointer to signal data
        uint32_t bits = 0;  // Signal width in bits
        uint32_t numWords = 0;  // Number of 32-bit words
        std::vector<uint32_t> prevValue;  // Previous value (for HD mode)
    };

    // MEMBERS
    VerilatedContext* m_contextp;  // Context pointer
    std::string m_triggerPath;  // Trigger signal: "scope.signame" or just "signame"
    std::string m_scopePath;  // Module scope path prefix
    bool m_hammingDistance;  // true = HD mode, false = HW mode
    std::string m_outputPrefix;  // Output file name prefix

    // Runtime state
    bool m_triggerPrev = false;  // Previous trigger value
    bool m_recording = false;  // Currently recording
    int m_fileIndex = 0;  // Sequential file counter
    FILE* m_fp = nullptr;  // Current CSV output file

    // Cached signal info
    SignalInfo m_trigger;  // Trigger signal info
    std::vector<SignalInfo> m_signals;  // All tracked signals
    bool m_initialized = false;

public:
    // CONSTRUCTORS

    /// Construct SCA tracer
    /// @param contextp  Verilated context
    /// @param trigger   Trigger signal hierarchical path (e.g. "TOP.dut.trigger")
    /// @param scope     Module scope for signal enumeration (e.g. "TOP.dut")
    /// @param type      "hw" for Hamming Weight, "hd" for Hamming Distance
    /// @param prefix    Output file prefix (default: "sca_trace")
    VerilatedSca(VerilatedContext* contextp, const std::string& trigger, const std::string& scope,
                 const std::string& type, const std::string& prefix = "sca_trace")
        : m_contextp{contextp}
        , m_triggerPath{trigger}
        , m_scopePath{scope}
        , m_hammingDistance{type == "hd"}
        , m_outputPrefix{prefix} {
        // Enable signal calculation even without waveform tracing
        contextp->traceEverOn(true);
    }

    ~VerilatedSca() { closeFile(); }

    // Prevent copying
    VerilatedSca(const VerilatedSca&) = delete;
    VerilatedSca& operator=(const VerilatedSca&) = delete;

    /// Sample all signals and record if trigger is high.
    /// Call this once per simulation timestep (after eval).
    void sample(uint64_t time) {
        if (VL_UNLIKELY(!m_initialized)) initialize();

        // Read trigger value
        const bool triggerNow = readTrigger();

        if (triggerNow && !m_triggerPrev) {
            // Rising edge: open new file
            openNewFile();
            m_recording = true;
        } else if (!triggerNow && m_triggerPrev) {
            // Falling edge: close file
            closeFile();
            m_recording = false;
        }
        m_triggerPrev = triggerNow;

        if (m_recording && m_fp) {
            // Compute and record trace value
            const uint64_t traceValue
                = m_hammingDistance ? computeHammingDistance() : computeHammingWeight();
            fprintf(m_fp, "%" PRIu64 ",%" PRIu64 "\n", time, traceValue);
        }
    }

private:
    // METHODS

    /// Initialize: resolve trigger and enumerate all signals in scope
    void initialize() {
        m_initialized = true;

        const VerilatedScopeNameMap* const scopeNameMapp = m_contextp->scopeNameMap();
        if (!scopeNameMapp || scopeNameMapp->empty()) {
            VL_FATAL_MT(__FILE__, __LINE__, "", "SCA: No scopes found. Is the model elaborated?");
            return;
        }

        // Resolve the trigger signal
        if (!resolveSignal(*scopeNameMapp, m_triggerPath, m_trigger)) {
            // Print available scopes to help the user debug
            VL_PRINTF_MT("%%Error: SCA: Cannot find trigger signal: %s\n", m_triggerPath.c_str());
            VL_PRINTF_MT("%%Error: SCA: Available scopes:\n");
            for (const auto& entry : *scopeNameMapp) {
                const VerilatedScope* sp = entry.second;
                VL_PRINTF_MT("%%Error: SCA:   scope '%s'", entry.first);
                const VerilatedVarNameMap* varsp = sp->varsp();
                if (varsp) { VL_PRINTF_MT(" (%zu vars)", varsp->size()); }
                VL_PRINTF_MT("\n");
            }
            VL_FATAL_MT(__FILE__, __LINE__, "",
                        ("SCA: Cannot find trigger signal: " + m_triggerPath).c_str());
            return;
        }

        // Enumerate all signals under the scope prefix
        enumerateSignals(*scopeNameMapp);

        if (m_signals.empty()) {
            VL_PRINTF_MT("%%Warning: SCA: No signals found under scope '%s'\n",
                         m_scopePath.c_str());
        } else {
            VL_PRINTF_MT("%%Info: SCA: Found %zu signals under scope '%s' (mode: %s)\n",
                         m_signals.size(), m_scopePath.c_str(),
                         m_hammingDistance ? "Hamming Distance" : "Hamming Weight");
        }

        // Initialize previous values for HD mode
        if (m_hammingDistance) {
            for (auto& sig : m_signals) {
                sig.prevValue.resize(sig.numWords, 0);
                readWords(sig.datap, sig.prevValue.data(), sig.numWords);
            }
        }
    }

    /// Check if a scope name matches a user-provided dotted path.
    /// Verilator scope names use the "pretty name" which should match
    /// the user-visible hierarchy. This does suffix matching to handle
    /// cases where the TOP prefix differs.
    static bool scopeMatches(const char* scopeName, const std::string& path) {
        // Exact match
        if (path == scopeName) return true;

        // Check if scopeName ends with the path (in case TOP prefix differs)
        const size_t scopeLen = std::strlen(scopeName);
        const size_t pathLen = path.size();
        if (scopeLen > pathLen && scopeName[scopeLen - pathLen - 1] == '.'
            && std::strcmp(scopeName + scopeLen - pathLen, path.c_str()) == 0) {
            return true;
        }

        return false;
    }

    /// Check if a scope name starts with a given prefix.
    /// Handles suffix matching for scope enumeration.
    static bool scopeStartsWith(const char* scopeName, const std::string& prefix) {
        const size_t scopeLen = std::strlen(scopeName);
        const size_t prefixLen = prefix.size();

        // Direct prefix match
        if (std::strncmp(scopeName, prefix.c_str(), prefixLen) == 0) {
            return scopeLen == prefixLen || scopeName[prefixLen] == '.';
        }

        // Suffix match: scope might have a longer prefix before our path
        // Look for ".prefix" or ".prefix." within scopeName
        const std::string dotPrefix = "." + prefix;
        const char* found = std::strstr(scopeName, dotPrefix.c_str());
        if (found) {
            const size_t afterMatch = (found - scopeName) + dotPrefix.size();
            return afterMatch == scopeLen || scopeName[afterMatch] == '.';
        }

        return false;
    }

    /// Resolve a hierarchical signal path into a SignalInfo.
    /// Tries multiple strategies:
    /// 1. Exact scope.var split at last dot
    /// 2. Search all scopes for the scope path, then find the var
    /// 3. Search all scopes for a variable matching the basename
    bool resolveSignal(const VerilatedScopeNameMap& scopeNameMap, const std::string& fullPath,
                       SignalInfo& info) {
        // Strategy 1: Split at last dot and try exact scopeFind
        const size_t lastDot = fullPath.rfind('.');
        if (lastDot != std::string::npos) {
            const std::string scopeName = fullPath.substr(0, lastDot);
            const std::string varName = fullPath.substr(lastDot + 1);

            // Try exact match first
            const VerilatedScope* scopep = m_contextp->scopeFind(scopeName.c_str());
            if (scopep) {
                const VerilatedVar* varp = scopep->varFind(varName.c_str());
                if (varp) {
                    info.datap = varp->datap();
                    info.bits = varp->entBits();
                    info.numWords = (info.bits + 31) / 32;
                    return true;
                }
            }

            // Strategy 2: Iterate all scopes, look for one matching the scope path
            for (const auto& entry : scopeNameMap) {
                if (!scopeMatches(entry.first, scopeName)) continue;
                const VerilatedVarNameMap* varsp = entry.second->varsp();
                if (!varsp) continue;
                const VerilatedVar* varp = entry.second->varFind(varName.c_str());
                if (varp) {
                    info.datap = varp->datap();
                    info.bits = varp->entBits();
                    info.numWords = (info.bits + 31) / 32;
                    return true;
                }
            }
        }

        // Strategy 3: Search all scopes for any variable with this basename
        const std::string basename
            = (lastDot != std::string::npos) ? fullPath.substr(lastDot + 1) : fullPath;
        for (const auto& entry : scopeNameMap) {
            const VerilatedVarNameMap* varsp = entry.second->varsp();
            if (!varsp) continue;
            for (const auto& varEntry : *varsp) {
                if (basename == varEntry.first) {
                    info.datap = varEntry.second.datap();
                    info.bits = varEntry.second.entBits();
                    info.numWords = (info.bits + 31) / 32;
                    VL_PRINTF_MT("%%Info: SCA: Resolved signal '%s' in scope '%s'\n",
                                 basename.c_str(), entry.first);
                    return true;
                }
            }
        }

        return false;
    }

    /// Enumerate all signals in scopes matching our scope prefix
    void enumerateSignals(const VerilatedScopeNameMap& scopeNameMap) {
        for (const auto& entry : scopeNameMap) {
            const char* scopeName = entry.first;
            const VerilatedScope* scopep = entry.second;

            // Check if scope matches our target prefix
            if (!scopeStartsWith(scopeName, m_scopePath)) continue;

            // Iterate all variables in this scope
            const VerilatedVarNameMap* varsp = scopep->varsp();
            if (!varsp) continue;

            for (const auto& varEntry : *varsp) {
                const VerilatedVar& var = varEntry.second;

                // Skip parameters
                if (var.isParam()) continue;

                // Skip unpacked arrays for now (focus on scalar/packed signals)
                if (var.udims() > 0) continue;

                SignalInfo sig;
                sig.datap = var.datap();
                sig.bits = var.entBits();
                sig.numWords = (sig.bits + 31) / 32;

                if (sig.datap && sig.bits > 0) { m_signals.push_back(std::move(sig)); }
            }
        }
    }

    /// Read the trigger signal value (returns true if nonzero)
    bool readTrigger() const {
        if (m_trigger.bits <= 32) {
            const uint32_t val = *reinterpret_cast<const uint32_t*>(m_trigger.datap);
            return (val & bitmask(m_trigger.bits)) != 0;
        }
        // Multi-word trigger: check any word nonzero
        const uint32_t* wp = reinterpret_cast<const uint32_t*>(m_trigger.datap);
        for (uint32_t i = 0; i < m_trigger.numWords; ++i) {
            uint32_t mask
                = (i == m_trigger.numWords - 1) ? bitmask(m_trigger.bits % 32) : 0xFFFFFFFFu;
            if (i == m_trigger.numWords - 1 && m_trigger.bits % 32 == 0) mask = 0xFFFFFFFFu;
            if (wp[i] & mask) return true;
        }
        return false;
    }

    /// Read signal words directly from memory
    static void readWords(const void* datap, uint32_t* words, uint32_t numWords) {
        const uint32_t* src = reinterpret_cast<const uint32_t*>(datap);
        for (uint32_t i = 0; i < numWords; ++i) words[i] = src[i];
    }

    /// Compute total Hamming Weight of all signals
    uint64_t computeHammingWeight() {
        uint64_t total = 0;

        for (const auto& sig : m_signals) {
            const uint32_t* wp = reinterpret_cast<const uint32_t*>(sig.datap);

            for (uint32_t i = 0; i < sig.numWords; ++i) {
                uint32_t word = wp[i];
                // Mask the top word to actual width
                if (i == sig.numWords - 1) {
                    const uint32_t topBits = sig.bits % 32;
                    if (topBits != 0) word &= bitmask(topBits);
                }
                total += popcount32(word);
            }
        }
        return total;
    }

    /// Compute total Hamming Distance (XOR with previous, then popcount)
    uint64_t computeHammingDistance() {
        uint64_t total = 0;

        for (auto& sig : m_signals) {
            const uint32_t* wp = reinterpret_cast<const uint32_t*>(sig.datap);

            for (uint32_t i = 0; i < sig.numWords; ++i) {
                uint32_t word = wp[i];
                // Mask the top word to actual width
                if (i == sig.numWords - 1) {
                    const uint32_t topBits = sig.bits % 32;
                    if (topBits != 0) word &= bitmask(topBits);
                }

                const uint32_t diff = word ^ sig.prevValue[i];
                total += popcount32(diff);

                // Update previous value
                sig.prevValue[i] = word;
            }
        }
        return total;
    }

    /// Bit mask for n bits (n must be 1..32, 0 returns all-ones)
    static uint32_t bitmask(uint32_t n) {
        return n == 0 ? 0xFFFFFFFFu : n >= 32 ? 0xFFFFFFFFu : (1u << n) - 1u;
    }

    /// Portable popcount for 32-bit values
    static uint32_t popcount32(uint32_t x) {
#if defined(__GNUC__) || defined(__clang__)
        return __builtin_popcount(x);
#else
        // Kernighan's bit counting
        uint32_t count = 0;
        while (x) {
            x &= x - 1;
            ++count;
        }
        return count;
#endif
    }

    /// Open a new sequentially numbered CSV file
    void openNewFile() {
        closeFile();
        const std::string filename = m_outputPrefix + "_" + std::to_string(m_fileIndex) + ".csv";
        m_fp = std::fopen(filename.c_str(), "w");
        if (!m_fp) {
            VL_PRINTF_MT("%%Error: SCA: Cannot open output file: %s\n", filename.c_str());
            return;
        }
        // Write CSV header
        fprintf(m_fp, "time,trace\n");
        ++m_fileIndex;
    }

    /// Close the current output file
    void closeFile() {
        if (m_fp) {
            std::fclose(m_fp);
            m_fp = nullptr;
        }
    }
};

#endif  // Guard

#include "runtime/ps2_pad.h"
#include "ps2_host_backend.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace
{
    constexpr uint8_t kPadAnalogMarker = 0x73;
    constexpr uint8_t kPadStickCenter = 0x80;

    constexpr uint16_t PAD_LEFT = 0x0080u;
    constexpr uint16_t PAD_DOWN = 0x0040u;
    constexpr uint16_t PAD_RIGHT = 0x0020u;
    constexpr uint16_t PAD_UP = 0x0010u;
    constexpr uint16_t PAD_START = 0x0008u;
    constexpr uint16_t PAD_R3 = 0x0004u;
    constexpr uint16_t PAD_L3 = 0x0002u;
    constexpr uint16_t PAD_SELECT = 0x0001u;
    constexpr uint16_t PAD_SQUARE = 0x8000u;
    constexpr uint16_t PAD_CROSS = 0x4000u;
    constexpr uint16_t PAD_CIRCLE = 0x2000u;
    constexpr uint16_t PAD_TRIANGLE = 0x1000u;
    constexpr uint16_t PAD_R1 = 0x0800u;
    constexpr uint16_t PAD_L1 = 0x0400u;
    constexpr uint16_t PAD_R2 = 0x0200u;
    constexpr uint16_t PAD_L2 = 0x0100u;

    // MOH test hook: PS2_MOH_AUTOPRESS="cross:200:60,start:600:60" presses buttons
    // automatically during [start, start+duration) readState calls (~1 call/frame),
    // so boot-flow tests can advance past "press X"/"press Start" screens headlessly.
    struct AutoPress
    {
        uint16_t mask;
        uint64_t from;
        uint64_t until;
    };

    const std::vector<AutoPress> &autoPressSchedule()
    {
        static const std::vector<AutoPress> schedule = [] {
            std::vector<AutoPress> out;
            const char *spec = std::getenv("PS2_MOH_AUTOPRESS");
            if (!spec)
                return out;
            char buf[512];
            std::snprintf(buf, sizeof(buf), "%s", spec);
            for (char *save = nullptr, *tok = strtok_r(buf, ",", &save); tok;
                 tok = strtok_r(nullptr, ",", &save))
            {
                char name[16] = {0};
                unsigned long from = 0, dur = 0;
                if (std::sscanf(tok, "%15[a-z]:%lu:%lu", name, &from, &dur) != 3)
                    continue;
                uint16_t mask = 0u;
                if (!std::strcmp(name, "cross")) mask = PAD_CROSS;
                else if (!std::strcmp(name, "start")) mask = PAD_START;
                else if (!std::strcmp(name, "circle")) mask = PAD_CIRCLE;
                else if (!std::strcmp(name, "triangle")) mask = PAD_TRIANGLE;
                else if (!std::strcmp(name, "square")) mask = PAD_SQUARE;
                else if (!std::strcmp(name, "up")) mask = PAD_UP;
                else if (!std::strcmp(name, "down")) mask = PAD_DOWN;
                if (mask)
                    out.push_back({mask, from, from + dur});
            }
            if (!out.empty())
                std::fprintf(stderr, "[MOH:autopress] %zu scheduled press(es)\n", out.size());
            return out;
        }();
        return schedule;
    }

    bool autoCrossPulseActive()
    {
        static const bool enabled = [] {
            const char *value = std::getenv("PS2_MOH_AUTO_CROSS");
            const bool active = value && !std::strcmp(value, "1");
            if (active)
                std::fprintf(stderr, "[MOH:auto-cross-enabled-v1]\n");
            return active;
        }();
        if (!enabled)
            return false;

        using Clock = std::chrono::steady_clock;
        static const auto started = Clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - started);
        return (elapsed.count() % 300) < 100;
    }
}

bool PSPadBackend::readState(int /*port*/, int /*slot*/, uint8_t *data, size_t size)
{
    if (!data || size < 32)
        return false;

    std::memset(data, 0, 32);
    data[0] = 0x01;
    data[1] = kPadAnalogMarker;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = data[5] = data[6] = data[7] = kPadStickCenter;

    uint16_t btns = 0xFFFFu;
    constexpr int kGamepad = 0;
    const bool useGamepad = IsGamepadAvailable(kGamepad);
    auto clearBit = [&btns](uint16_t mask)
    { btns &= ~mask; };

    if (useGamepad)
    {
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_UP))
            clearBit(PAD_UP);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_DOWN))
            clearBit(PAD_DOWN);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_LEFT))
            clearBit(PAD_LEFT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT))
            clearBit(PAD_RIGHT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_DOWN))
            clearBit(PAD_CROSS);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT))
            clearBit(PAD_CIRCLE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_LEFT))
            clearBit(PAD_SQUARE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_UP))
            clearBit(PAD_TRIANGLE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1))
            clearBit(PAD_L1);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1))
            clearBit(PAD_R1);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_2))
            clearBit(PAD_L2);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_2))
            clearBit(PAD_R2);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_MIDDLE_RIGHT))
            clearBit(PAD_START);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_MIDDLE_LEFT))
            clearBit(PAD_SELECT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_THUMB))
            clearBit(PAD_L3);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_THUMB))
            clearBit(PAD_R3);

        float lx = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_LEFT_X);
        float ly = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_LEFT_Y);
        float rx = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_RIGHT_X);
        float ry = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_RIGHT_Y);
        data[6] = static_cast<uint8_t>(128 + lx * 127);
        data[7] = static_cast<uint8_t>(128 + ly * 127);
        data[4] = static_cast<uint8_t>(128 + rx * 127);
        data[5] = static_cast<uint8_t>(128 + ry * 127);
    }

    // Keyboard remains active even when a physical gamepad is detected.
    {
        // Digital directions.
        if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W))
            clearBit(PAD_UP);
        if (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S))
            clearBit(PAD_DOWN);
        if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))
            clearBit(PAD_LEFT);
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))
            clearBit(PAD_RIGHT);

        // Face buttons.
        if (IsKeyDown(KEY_X) || IsKeyDown(KEY_SPACE))
            clearBit(PAD_CROSS);
        if (IsKeyDown(KEY_C) || IsKeyDown(KEY_ESCAPE))
            clearBit(PAD_CIRCLE);
        if (IsKeyDown(KEY_Z) || IsKeyDown(KEY_KP_0))
            clearBit(PAD_SQUARE);
        if (IsKeyDown(KEY_V) || IsKeyDown(KEY_KP_1))
            clearBit(PAD_TRIANGLE);

        // Shoulder buttons.
        if (IsKeyDown(KEY_Q))
            clearBit(PAD_L1);
        if (IsKeyDown(KEY_E))
            clearBit(PAD_R1);
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_ONE))
            clearBit(PAD_L2);
        if (IsKeyDown(KEY_RIGHT_SHIFT) || IsKeyDown(KEY_THREE))
            clearBit(PAD_R2);

        // System and stick buttons.
        if (IsKeyDown(KEY_ENTER))
            clearBit(PAD_START);
        if (IsKeyDown(KEY_TAB))
            clearBit(PAD_SELECT);
        if (IsKeyDown(KEY_U))
            clearBit(PAD_L3);
        if (IsKeyDown(KEY_O))
            clearBit(PAD_R3);

        // Left analog stick: I/J/K/L.
        const bool leftStickLeft = IsKeyDown(KEY_J);
        const bool leftStickRight = IsKeyDown(KEY_L);
        const bool leftStickUp = IsKeyDown(KEY_I);
        const bool leftStickDown = IsKeyDown(KEY_K);

        if (leftStickLeft != leftStickRight)
            data[6] = leftStickLeft ? 0x00 : 0xFF;
        if (leftStickUp != leftStickDown)
            data[7] = leftStickUp ? 0x00 : 0xFF;

        // Right analog stick: T/F/G/H.
        const bool rightStickLeft = IsKeyDown(KEY_F);
        const bool rightStickRight = IsKeyDown(KEY_H);
        const bool rightStickUp = IsKeyDown(KEY_T);
        const bool rightStickDown = IsKeyDown(KEY_G);

        if (rightStickLeft != rightStickRight)
            data[4] = rightStickLeft ? 0x00 : 0xFF;
        if (rightStickUp != rightStickDown)
            data[5] = rightStickUp ? 0x00 : 0xFF;
    }

    {
        const auto &schedule = autoPressSchedule();
        if (!schedule.empty())
        {
            static uint64_t s_readCount = 0u;
            const uint64_t now = s_readCount++;
            for (const auto &ap : schedule)
            {
                if (now >= ap.from && now < ap.until)
                {
                    if (now == ap.from)
                        std::fprintf(stderr, "[MOH:autopress] press mask=0x%04x at read=%llu\n",
                                     ap.mask, static_cast<unsigned long long>(now));
                    clearBit(ap.mask);
                }
            }
        }
    }

    if (autoCrossPulseActive())
        clearBit(PAD_CROSS);

    data[2] = static_cast<uint8_t>(btns & 0xFF);
    data[3] = static_cast<uint8_t>(btns >> 8);
    return true;
}

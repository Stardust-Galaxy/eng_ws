
#pragma once
#include <cstdint>

namespace RM_referee {
    enum class PacketType : uint16_t {
        CustomRobotData = 0x0302,
        KeyboardMouseMessage = 0x0304,
    };

    enum class GameType : uint8_t {
        Standard = 1,
        RMUTechnicalChallenge,
        ICRA_RMUA,
        RMUL_3V3,
        RMUL_1V1
    };

    enum class GameStage : uint8_t {
        NotStarted = 0,
        Preparing,
        SelfTest,
        Countdown,
        Started,
        EndScreen,
    };

    enum class GameResult : uint8_t {
        Tie = 0,
        RedWin,
        BlueWin,
    };

    enum class RobotID : uint8_t {
        Red1 = 1,
        Red2 = 2,
        Red3 = 3,
        Red4 = 4,
        Red5 = 5,
        RedAerial = 6,
        RedSentry = 7,
        RedDart = 8,
        RedRadar = 9,

        Blue1 = 101,
        Blue2 = 102,
        Blue3 = 103,
        Blue4 = 104,
        Blue5 = 105,
        BlueAerial = 106,
        BlueSentry = 107,
        BlueDart = 108,
        BlueRadar = 109,
    };

    enum class GraphicsType : uint8_t {
        Line = 0,
        Rectangle,
        Circle,
        Ellipse,
        FpNumber,
        IntNumber,
        String,
    };

    enum class GraphicsColor : uint8_t {
        TeamColor = 0,
        Yellow,
        Green,
        Orange,
        Violet,
        Pink,
        Cyan,
        Black,
        White,
    };
};

#ifndef OTSIM_S7_UTILS_HPP
#define OTSIM_S7_UTILS_HPP

#include <cstdint>
#include <cstring>

/*
 * S7 uses big-endian for multi-byte values.
 * These utilities ensure proper byte-order conversion between host
 * and S7 big-endian format, ensuring compatibility with physical Siemens S7 PLCs.
 *
 * IEC 61131-3 also uses data types: bool, byte, word, dword, int, dint, and reals (and more
 * but those are some common ones)
 * reference: https://library.e.abb.com/public/81478a314e1386d1c1257b1a005b0fc0/2101127.pdf
 * 
 * IEC 61131-3 also supports bit-level addressing
 *
 * These helper functions handle these operations
 */

namespace otsim {
namespace s7 {
namespace utils {


enum class S7DataType : uint8_t {
    BOOL,
    BYTE,
    WORD,
    DWORD,
    INT,
    DINT,
    REAL
};

// Byte order detection at compile time
constexpr bool isLittleEndian() {
    // check endianness
    #if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        return false;
    #elif defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        return true;
    #else
        // default assumption for x86/x64
        return true;
    #endif
}

// byte swap functions
inline uint16_t byteSwap16(uint16_t value) {
    return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

inline uint32_t byteSwap32(uint32_t value) {
    return ((value & 0xFF000000) >> 24) |
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0x0000FF00) << 8)  |
           ((value & 0x000000FF) << 24);
}

// convert host 16-bit value to big-endian
inline uint16_t hostToS7Word(uint16_t value) {
    if constexpr (isLittleEndian()) {
        return byteSwap16(value);
    }
    return value;
}

// convert S7 big-endian 16-bit value to host format
inline uint16_t s7ToHostWord(uint16_t value) {
    return hostToS7Word(value);
}

// convert host 32-bit value to big-endian
inline uint32_t hostToS7DWord(uint32_t value) {
    if constexpr (isLittleEndian()) {
        return byteSwap32(value);
    }
    return value;
}

// convert S7 big-endian 32-bit value to host format
inline uint32_t s7ToHostDWord(uint32_t value) {
    return hostToS7DWord(value);
}

// convert host float to S7 big-endian IEEE 754 format
inline float hostToS7Real(float value) {
    if constexpr (isLittleEndian()) {
        uint32_t temp;
        std::memcpy(&temp, &value, sizeof(float));
        temp = byteSwap32(temp);
        float result;
        std::memcpy(&result, &temp, sizeof(float));
        return result;
    }
    return value;
}

// convert S7 big-endian float to host format
inline float s7ToHostReal(float value) {
    return hostToS7Real(value); 
}

// convert host signed 16-bit to S7 format
inline int16_t hostToS7Int(int16_t value) {
    uint16_t temp = static_cast<uint16_t>(value);
    temp = hostToS7Word(temp);
    return static_cast<int16_t>(temp);
}

// convert S7 signed 16-bit to host format
inline int16_t s7ToHostInt(int16_t value) {
    return hostToS7Int(value);
}

// convert host signed 32-bit to S7 format
inline int32_t hostToS7DInt(int32_t value) {
    uint32_t temp = static_cast<uint32_t>(value);
    temp = hostToS7DWord(temp);
    return static_cast<int32_t>(temp);
}

// convert S7 signed 32-bit to host format
inline int32_t s7ToHostDInt(int32_t value) {
    return hostToS7DInt(value); 
}

/*
 * S7 uses bit-level addressing for BOOL types
 *
 *   M0.0 → address = 0  (byte 0, bit 0)
 *   M0.7 → address = 7  (byte 0, bit 7)
 *   
 * it's only for bools because bools are 1 bit large (so if we are changing
 * 1 bit it can't simulate anything besides a bool)
 */

// extract byte offset from bit-packed address
inline uint16_t getByteOffset(uint16_t bitAddress) {
    // divide by 8
    return bitAddress >> 3;
}

// extract bit offset from bit-packed address
inline uint8_t getBitOffset(uint16_t bitAddress) {
    // modulo 8
    return bitAddress & 0x07;
}

// encode byte and bit offsets into a single address
inline uint16_t encodeBitAddress(uint16_t byteOffset, uint8_t bitOffset) {
    return (byteOffset << 3) | (bitOffset & 0x07);
}

// write a single bit to S7 buffer
inline bool writeBit(uint8_t* buffer, size_t bufLen, uint16_t bitAddress, bool value) {
    uint16_t byteOffset = getByteOffset(bitAddress);
    uint8_t bitOffset = getBitOffset(bitAddress);

    if (byteOffset >= bufLen) {
        return false;
    }

    if (value) { // set the bit
        buffer[byteOffset] |= (1 << bitOffset);
    } else { // clear the bit
        buffer[byteOffset] &= ~(1 << bitOffset);
    }

    return true;
}

// read a bit from S7 buffer
inline bool readBit(const uint8_t* buffer, size_t bufLen, uint16_t bitAddress, bool& value) {
    uint16_t byteOffset = getByteOffset(bitAddress);
    uint8_t bitOffset = getBitOffset(bitAddress);

    // if out of bounds
    if (byteOffset >= bufLen) {
        return false;
    }

    value = (buffer[byteOffset] & (1 << bitOffset)) != 0;
    return true;
}

// write full byte to S7 buffer
inline bool writeByte(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint8_t value) {
    if (byteAddr >= bufLen) {
        return false;
    }
    buffer[byteAddr] = value;
    return true;
}

// read full byte from S7 buffer
inline bool readByte(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint8_t& value) {
    if (byteAddr >= bufLen) {
        return false;
    }
    value = buffer[byteAddr];
    return true;
}

// write a word to the buffer
inline bool writeWord(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint16_t value) {
    if (byteAddr + sizeof(uint16_t) > bufLen) {
        return false;
    }
    uint16_t s7Value = hostToS7Word(value);
    std::memcpy(&buffer[byteAddr], &s7Value, sizeof(uint16_t));
    return true;
}

// read a word from the buffer
inline bool readWord(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint16_t& value) {
    if (byteAddr + sizeof(uint16_t) > bufLen) {
        return false;
    }
    uint16_t s7Value;
    std::memcpy(&s7Value, &buffer[byteAddr], sizeof(uint16_t));
    value = s7ToHostWord(s7Value);
    return true;
}

// write a dword to the buffer
inline bool writeDWord(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint32_t value) {
    if (byteAddr + sizeof(uint32_t) > bufLen) {
        return false;
    }
    uint32_t s7Value = hostToS7DWord(value);
    std::memcpy(&buffer[byteAddr], &s7Value, sizeof(uint32_t));
    return true;
}

// read a dword from the buffer
inline bool readDWord(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, uint32_t& value) {
    if (byteAddr + sizeof(uint32_t) > bufLen) {
        return false;
    }
    uint32_t s7Value;
    std::memcpy(&s7Value, &buffer[byteAddr], sizeof(uint32_t));
    value = s7ToHostDWord(s7Value);
    return true;
}

// write an int to the buffer
inline bool writeInt(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, int16_t value) {
    if (byteAddr + sizeof(int16_t) > bufLen) {
        return false;
    }
    int16_t s7Value = hostToS7Int(value);
    std::memcpy(&buffer[byteAddr], &s7Value, sizeof(int16_t));
    return true;
}

// read an int from the buffer
inline bool readInt(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, int16_t& value) {
    if (byteAddr + sizeof(int16_t) > bufLen) {
        return false;
    }
    int16_t s7Value;
    std::memcpy(&s7Value, &buffer[byteAddr], sizeof(int16_t));
    value = s7ToHostInt(s7Value);
    return true;
}

// write dint to the buffer
inline bool writeDInt(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, int32_t value) {
    if (byteAddr + sizeof(int32_t) > bufLen) {
        return false;
    }
    int32_t s7Value = hostToS7DInt(value);
    std::memcpy(&buffer[byteAddr], &s7Value, sizeof(int32_t));
    return true;
}

// read a dint from the buffer
inline bool readDInt(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, int32_t& value) {
    if (byteAddr + sizeof(int32_t) > bufLen) {
        return false;
    }
    int32_t s7Value;
    std::memcpy(&s7Value, &buffer[byteAddr], sizeof(int32_t));
    value = s7ToHostDInt(s7Value);
    return true;
}

// write a real to the buffer
inline bool writeReal(uint8_t* buffer, size_t bufLen, uint16_t byteAddr, float value) {
    if (byteAddr + sizeof(float) > bufLen) {
        return false;
    }
    float s7Value = hostToS7Real(value);
    std::memcpy(&buffer[byteAddr], &s7Value, sizeof(float));
    return true;
}

// read a real from the buffer
inline bool readReal(const uint8_t* buffer, size_t bufLen, uint16_t byteAddr, float& value) {
    if (byteAddr + sizeof(float) > bufLen) {
        return false;
    }
    float s7Value;
    std::memcpy(&s7Value, &buffer[byteAddr], sizeof(float));
    value = s7ToHostReal(s7Value);
    return true;
}

} // namespace utils
} // namespace s7
} // namespace otsim

#endif // OTSIM_S7_UTILS_HPP

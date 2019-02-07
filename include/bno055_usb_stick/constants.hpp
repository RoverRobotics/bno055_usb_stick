#ifndef BNO055_USB_STICK_CONSTANTS_HPP
#define BNO055_USB_STICK_CONSTANTS_HPP

#include <cstddef>

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {

struct Constants {
  // Length of a data stream packet
  enum { DAT_LEN = 0x38, HDR_LEN = 5 };

  // Adafruit Board Constants
  //# BOSCH BNO055 IMU Registers map and other information
  enum { 
    // Page 0 registers
    CHIP_ID = 0x00,
    PAGE_ID = 0x07,
    ACCEL_DATA = 0x08,
    MAG_DATA = 0x0e,
    GYRO_DATA = 0x14,
    FUSED_EULER = 0x1a,
    FUSED_QUAT = 0x20,
    LIA_DATA = 0x28,
    GRAVITY_DATA = 0x2e,
    TEMP_DATA = 0x34,
    CALIB_STAT = 0x35,
    SYS_STATUS = 0x39,
    SYS_ERR = 0x3a,
    UNIT_SEL = 0x3b,
    OPER_MODE = 0x3d,
    PWR_MODE = 0x3e,
    SYS_TRIGGER = 0x3f,
    TEMP_SOURCE = 0x440,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,

    ACC_OFFSET = 0x55,
    MAG_OFFSET = 0x5b,
    GYR_OFFSET = 0x61,
    ACC_RADIUS = 0x68,
    MAG_RADIUS = 0x69,

    // Page 1 registers
    ACC_CONFIG = 0x08,
    MAG_CONFIG = 0x09,
    GYR_CONFIG0 = 0x0a,
    GYR_CONFIG1 = 0x0b,

    //  Operation modes
    OPER_MODE_CONFIG = 0x00,
    OPER_MODE_ACCONLY = 0x01,
    OPER_MODE_MAGONLY = 0x02,
    OPER_MODE_GYROONLY = 0x03,
    OPER_MODE_ACCMAG = 0x04,
    OPER_MODE_ACCGYRO = 0x05,
    OPER_MODE_MAGGYRO = 0x06,
    OPER_MODE_AMG = 0x07,
    //Fusion Modes (these provide orientation data)
    OPER_MODE_IMU = 0x08,
    OPER_MODE_COMPASS = 0x09,
    OPER_MODE_M4G = 0x0a,
    OPER_MODE_NDOF_FMC_OFF = 0x0b,
    OPER_MODE_NDOF = 0x0C,

    //FUSION_MODES = [OPER_MODE_IMU, OPER_MODE_COMPASS, OPER_MODE_M4G, OPER_MODE_NDOF_FMC_OFF, OPER_MODE_NDOF],

    //  Power modes
    PWR_MODE_NORMAL = 0x00,
    PWR_MODE_LOW = 0x01,
    PWR_MODE_SUSPEND  = 0x02,

    // Communication constants
    BNO055_ID = 0xa0,
    START_BYTE_WR = 0xaa,
    START_BYTE_RESP = 0xbb,
    READ = 0x01,
    WRITE = 0x00
  };

  // Form of a data stream packet
  enum {
    HDR_POS = 0,
    BDY_POS = HDR_POS + HDR_LEN,
    ACC_POS = BDY_POS,
    MAG_POS = ACC_POS + 6,
    GYR_POS = MAG_POS + 6,
    EUL_POS = GYR_POS + 6,
    QUA_POS = EUL_POS + 6,
    LIA_POS = QUA_POS + 8,
    GRV_POS = LIA_POS + 6,
    TEMP_POS = GRV_POS + 6,
    CALIB_STAT_POS = TEMP_POS + 1
  };

  // Denominators to convert a byte/word to a real value
  enum {
    ACC_DENOM = 100,
    MAG_DENOM = 16,
    GYR_DENOM = 16,
    EUL_DENOM = 16,
    QUA_DENOM = 1 << 14,
    LIA_DENOM = 100,
    GRV_DENOM = 100,
    TEMP_DENOM = 1
  };

  static std::size_t getCommandLength(const boost::uint8_t *command) { return command[1]; }

  static const boost::uint8_t **toNDOFCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x3d, 0x00, 0x01, 0x01, 0x00, 0x01, 0x1c, 0x0d, 0x0a}; //0x1c vs 
    static const boost::uint8_t *commands[] = {_00, _01, NULL};
    return commands;
  }

  static const boost::uint8_t **toIMUCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x3d, 0x00, 0x01, 0x01, 0x00, 0x01, 0x18, 0x0d, 0x0a}; //0x18
    static const boost::uint8_t *commands[] = {_00, _01, NULL};
    return commands;
  }
/*
    buf_out = bytearray()
    buf_out.append(START_BYTE_WR)
    buf_out.append(WRITE)
    buf_out.append(reg_addr)
    buf_out.append(length)
    buf_out.append(data)
*/
  static const boost::uint8_t **toIMUCommandsAdafruit() {
    static const boost::uint8_t _00[] = {START_BYTE_WR, WRITE, OPER_MODE, 0x01, 0x08};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x3d, 0x00, 0x01, 0x01, 0x00, 0x01, 0x18, 0x0d, 0x0a}; //0x18
    static const boost::uint8_t *commands[] = {_00, _01, NULL};
    return commands;
  }

  static const boost::uint8_t **startStreamCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x06, 0x06, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _02[] = {0xaa, 0x0a, 0x03, 0x02, 0x01,
                                         0x00, 0x0a, 0x02, 0x0d, 0x0a};
    static const boost::uint8_t _03[] = {0xaa, 0x14, 0x04, 0x01, 0x0e, 0x00, 0x00,
                                         0x00, 0x28, 0x00, 0x0a, 0x02, 0x08, 0x12,
                                         0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _04[] = {0xaa, 0x15, 0x04, 0x02, 0x0e, 0x00, 0x00,
                                         0x00, 0x28, 0x00, 0x0a, 0x02, 0x1a, 0x1e,
                                         0x00, 0x00, 0x01, 0x07, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _05[] = {0xaa, 0x06, 0x06, 0xff, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, _01, _02, _03, _04, _05, NULL};
    return commands;
  }

  static const boost::uint8_t **stopStreamCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x06, 0x06, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, NULL};
    return commands;
  }
};
}

#endif // BNO055_USB_STICK_CONSTANTS_HPP
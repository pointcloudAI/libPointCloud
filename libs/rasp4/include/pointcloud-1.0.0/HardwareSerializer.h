/*
 * PointCloud Lib component.
 *
 * Copyright (c)  PointCloud.AI Inc.
 */

#include "Common.h"
#include "USBIO.h"
#include "SerializedObject.h"

/**
 * \ingroup Util
 */

#define ERASE_EEPROM_DATA 0x42
#define SEARIAL_CMD 0x44
#define EEPROM_PAGE_SIZE 256
#define BASE_PAGE_OFFSET 0
#define BASE_SECTION_OFFSET 0
namespace PointCloud
{
 
class POINTCLOUD_EXPORT HardwareSerializer
{
  uint8_t _ioRequestCode, _sizeRequestCode;
  USBIOPtr _usbIO;
  bool isUseCRC;
public:
  HardwareSerializer() {}
  
  /**
   * 
   * @param usbIO -- Pointer to USBIO class object which supports control transfers
   * @param ioRequestCode -- request code to read/write serialized data from/to hardware
   * @param sizeRequestCode -- request code to read size of serialized data in hardware
   * 
   */
  HardwareSerializer(USBIOPtr &usbIO, uint8_t ioRequestCode, uint8_t sizeRequestCode): 
    _usbIO(usbIO), _ioRequestCode(ioRequestCode), _sizeRequestCode(sizeRequestCode)
    {
        isUseCRC = false;
    }
  
    bool checkCRC(Version &version)
    {
        if(version.major == 0 && version.minor == 1)
            return false;
        else
            return true;
    }
  /**
   * @param version is the version info of format of 'so' 
   * @param knownTimestamp is the last known timestamp of data written. This is used to see whether an exist local copy of the 
   * serialized data is same as that in hardware or not. If same, then nothing is read from hardware improving run-time performance.
   * @param so is the object to hold serialized data to read from hardware
   */
  bool read(Version &version, TimeStampType &knownTimestamp, SerializedObject &so);
  bool readPage(uint16_t base_page,uint8_t* data,uint16_t length,int max_try=4);

  /**
   * @param version is the version info of format of 'so' 
   * @param timestamp is the timestamp of data to be written. This is used by read() to see whether an exist local copy of the 
   * serialized data is same as that in hardware or not.
   * @param so is the object holding serialized data to write to hardware
   */
  bool write(Version &version, TimeStampType &timestamp, SerializedObject &so);
  
  bool writeSerialNumber(std::string serialNumber);
  bool getSerialNumber(std::string& serialNumber);
  /**
   *  Save to local file 'filename'
   */
  bool writeToFile(const String &filename, Version &version, TimeStampType &timestamp, SerializedObject &so);
  
  inline void setUSBIO(USBIOPtr &usbIO) { _usbIO = usbIO; }
  
  inline operator bool () { return _usbIO.get() != nullptr; }
  
  /**
   * Serialized size in hardware is assumed to be 4 bytes long in big endian format
   */
  bool getSize(uint32_t &size);
  
  virtual ~HardwareSerializer() {}
};

typedef Ptr<HardwareSerializer> HardwareSerializerPtr;
  
}

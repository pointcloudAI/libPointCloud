/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_DEPTHCAMERA_H
#define POINTCLOUD_DEPTHCAMERA_H

#include <Device.h>
#include <Frame.h>
#include "VideoMode.h"
#include <Filter/FilterSet.h>
#include "FrameStream.h"
#include "Timer.h"
#include "Configuration.h"

#define SPEED_OF_LIGHT 299792458.0
#define BULK_XFER_EXTRA_SIZE  255
#define EBD_RAW12_DATA_LENGTH 396
#define EBD_INT16_DATA_LENGTH 528

#define UNAMBIGUOUS_RANGE "unambiguous_range"
#define NEAR_DISTANCE "near_distance"
#define MEASURE_MODE "measure_mode"
#define FILTER_EN "filter_en"
#define INTG_SCALE "intg_scale"
#define INTG_TIME "intg_time"

#define CALIB_SECT_LENS "lens"
#define CALIB_SECT_LENS_ID 0
#define TSENSOR "tsensor"
#define TILLUM  "tillum"
#define EBD_DEPTH_MAP_ID_OFFSET 87
#define EBD_FRAME_COUNT_OFFSET 63
#define EBD_SUB_FRAME_ID_OFFSET 54
#define EBD_TSENSOR_START_OFFSET 117
#define EBD_TILLUM_OFFSET 2
#define EBD_POWER_SUPPLIER 3

namespace PointCloud
{
    class POINTCLOUD_EXPORT Streamer;
    class POINTCLOUD_EXPORT Parameter;
    class POINTCLOUD_EXPORT FrameGenerator;
    class POINTCLOUD_EXPORT PointCloudFrameGenerator;
    class POINTCLOUD_EXPORT RegisterProgrammer;
    typedef Ptr<Streamer>  StreamerPtr;
    typedef Ptr<Parameter>  ParameterPtr;
    typedef Ptr<RegisterProgrammer> RegisterProgrammerPtr;
    typedef Ptr<FrameGenerator> FrameGeneratorPtr;
    typedef Ptr<PointCloudFrameGenerator> PointCloudFrameGeneratorPtr;
    /**
      * \ingroup CamSys
      *
      * \brief This is primary class which provides API for a depth camera.
      *
      * DepthCamera is an abstract class which needs to be derived and implemented
      * for individual depth camera types.
      */
    class POINTCLOUD_EXPORT DepthCamera
    {
    public:
      enum FrameType
      {
        FRAME_RAW_FRAME_UNPROCESSED = 0,
        FRAME_RAW_FRAME_PROCESSED = 1,
        FRAME_DEPTH_FRAME = 2,
        FRAME_XYZI_POINT_CLOUD_FRAME = 3,
        FRAME_TYPE_COUNT = 4 // This is just used for number of callback types
      };
      typedef Function<void (DepthCamera &camera, const Frame &frame, FrameType callBackType)> CallbackType;
      typedef Function<void (DepthCamera &camera, int isOn12V)> PowerChangedCallback;
    public:
        DepthCamera(const String &name, const String &chipset, DevicePtr device);
        virtual ~DepthCamera();
        virtual bool isInit() = 0;
        virtual bool isInitialized() const;
        inline bool isRunning() const { return _running; }
        inline bool isPaused() const { return _isPaused; }
        bool start();
        bool stop();
        void wait();
        bool reset();
        bool pause();
        bool resume();
        bool close();
        
        virtual bool registerCallback(FrameType type, CallbackType f);
        virtual bool clearAllCallbacks();
        virtual bool clearCallback(FrameType type);
         bool isRawDataProvidedOnly();

        virtual bool registerPowerSupplierChangedCallback(PowerChangedCallback f);
        virtual bool setPowerLevel(int level) =0;
        
        bool setFrameRate(const FrameRate &r);
        bool getFrameRate(FrameRate &r) const;
        bool setFrameSize(const FrameSize &s);
        bool getFrameSize(FrameSize &s) const;
        bool getMaximumFrameSize(FrameSize &s) const;
        bool getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const;
        bool getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const;
        bool getMaximumVideoMode(VideoMode &videoMode) const;
        
        bool getROI(RegionOfInterest &roi);
        bool setROI(const RegionOfInterest &roi);
        bool allowedROI(String &message);
        
        bool getFieldOfView(float &fovHalfAngle) const;
        
        virtual bool saveFrameStream(const String &fileName);
        virtual bool isSavingFrameStream();
        virtual bool closeFrameStream();
        
        template <typename T>
        bool get(const String &name, T &value, bool refresh = false) const;
        template <typename T>
        bool set(const String &name, const T &value);
        
        inline const String &name() const { return _name; }
        inline const String &id() const { return _id; }
        inline const String &chipset() const { return _chipset; }
    
        virtual bool getSerialNumber(String &serialNumber) const;
        virtual bool setSerialNumber(const String &serialNumber);
        
        inline const DevicePtr &getDevice() const { return _device; }
        template <typename T>
        bool getStreamParam(const String &name, T &value) const;
        bool refreshParams();
        
        // WARNING: Avoid using get() and set() on ParameterPtr, obtained via getParam() or getParameters(). It is not thread-safe. Instead use get() and set() on DepthCamera
        const ParameterPtr getParam(const String &name) const;
        inline const Map<String, ParameterPtr> &getParameters() const { return _parameters; }
        
        bool getBytesPerPixel(uint &bpp) const;
        bool setBytesPerPixel(const uint &bpp);
        bool getFrameGeneratorConfig(FrameType type, SerializedObject &object);
        // beforeFilterIndex = -1 => at the end, otherwise at location before the given filter index.
        // Return value:
        //   >= 0 => add successfully with return value as filter ID.
        //   -1 => failed to add filter
        virtual int addFilter(FilterPtr p, FrameType frameType, int beforeFilterID = -1);
        virtual FilterPtr getFilter(int filterID, FrameType frameType) const;
        virtual bool removeFilter(int filterID, FrameType frameType);
        virtual bool removeAllFilters(FrameType frameType);
        virtual void resetFilters();
        
        inline const FilterSet<RawFrame> &getUnprocessedRawFilterSet() { return _unprocessedFilters; }
        inline const FilterSet<RawFrame> &getProcessedRawFilterSet() { return _processedFilters; }
        inline const FilterSet<DepthFrame> &getDepthFilterSet() { return _depthFilters; }
        
        // RegisterProgrammer is usually thread-safe to use outside directly
        inline RegisterProgrammerPtr getProgrammer() { return _programmer; }
        // Streamer may not be thread-safe
        inline StreamerPtr getStreamer() { return _streamer; }
        
        inline bool reloadConfiguration() { return configFile.read(_name + ".conf"); }
        inline const Map<int, String> &getCameraProfileNames() { return configFile.getCameraProfileNames(); }
        inline int getCurrentCameraProfileID() { return configFile.getCurrentProfileID(); }
        
        int  addCameraProfile(const String &profileName, const int parentID);
        bool setCameraProfile(const int id, bool softApply = false);
        bool removeCameraProfile(const int id);

        virtual bool _onPowerChangedCallback(int isOn12V);
        

        inline bool saveCameraProfileToHardware(int &id, bool saveParents = false, bool setAsDefault = false, const String &namePrefix = "") { return configFile.saveCameraProfileToHardware(id, saveParents, setAsDefault, namePrefix); }
        
    protected:
          bool _addParameters(const Vector<ParameterPtr> &params);
          // Callback the registered function for 'type' if present and decide whether continue processing or not
          virtual bool _callbackAndContinue(uint32_t &callBackTypesToBeCalled, FrameType type, const Frame &frame);
        
        
          bool _init();
          virtual bool _start() = 0;
          virtual bool _stop() = 0;
          virtual bool _close() = 0;
        
          virtual bool _captureRawUnprocessedFrame(RawFramePtr &rawFrame) = 0;
          virtual bool _processRawFrame(const RawFramePtr &rawFrameInput, RawFramePtr &rawFrameOutput) = 0; // here output raw frame will have processed data, like ToF data for ToF cameras
          virtual bool _convertToDepthFrame(const RawFramePtr &rawFrame, DepthFramePtr &depthFrame) = 0;
          virtual bool _convertToPointCloudFrame(const DepthFramePtr &depthFrame, PointCloudFramePtr &pointCloudFrame);
        
          virtual void _captureLoop(); // the main capture loop
          void _captureThreadWrapper(); // this is non-virtual and simply calls _captureLoop
          bool _running, _isPaused; // is capture running?
          bool _writeToFrameStream(RawFramePtr &rawUnprocessed);
        
          // These protected getters and setters are not thread-safe. These are to be directly called only when nested calls are to be done from getter/setter to another.
          // Otherwise use the public functions
          template <typename T>
          bool _get(const String &name, T &value, bool refresh = false) const;
          template <typename T>
          bool _set(const String &name, const T &value);
        
          virtual bool _setFrameRate(const FrameRate &r) = 0;
          virtual bool _getFrameRate(FrameRate &r) const = 0;
          virtual bool _setFrameSize(const FrameSize &s) = 0;
          virtual bool _getFrameSize(FrameSize &s) const = 0;
          virtual bool _getMaximumFrameSize(FrameSize &s) const = 0;
          virtual bool _getMaximumFrameRate(FrameRate &frameRate, const FrameSize &forFrameSize) const = 0;
          virtual bool _getSupportedVideoModes(Vector<SupportedVideoMode> &supportedVideoModes) const = 0;
          virtual bool _getMaximumVideoMode(VideoMode &videoMode) const = 0;
        
          virtual bool _getBytesPerPixel(uint &bpp) const = 0;
          virtual bool _setBytesPerPixel(const uint &bpp) = 0;
        
          virtual bool _getROI(RegionOfInterest &roi) const = 0;
          virtual bool _setROI(const RegionOfInterest &roi) = 0;
          virtual bool _allowedROI(String &message) = 0;
          virtual bool _getFieldOfView(float &fovHalfAngle) const = 0;
          inline void _makeID() { _id = _name + "(" + _device->id() + ")"; }
          virtual bool _reset() = 0;
          virtual bool _onReset() = 0;
          virtual bool _applyConfigParams(const ConfigSet *params);
          virtual bool _saveCurrentProfileID(const int id) = 0;
          virtual bool _getCurrentProfileID(int &id) = 0;
          inline Map<String, CalibrationInformation> &_getCalibrationInformationStructure() { return configFile._calibrationInformation; }
    public:
        MainConfigurationFile configFile; // This corresponds to camera specific configuration file
    private:
        mutable Mutex _accessMutex; // This is locked by getters and setters which are public
        mutable Mutex _frameStreamWriterMutex;
    protected:
        DevicePtr _device;
        String    _name, _id, _chipset;
        
        Map<String, ParameterPtr> _parameters;
        RegisterProgrammerPtr     _programmer;
        StreamerPtr               _streamer;
        
        FrameGeneratorPtr _frameGenerators[3];
        PointCloudFrameGeneratorPtr _pointCloudFrameGenerator;
        bool _parameterInit;
        
        RawFramePtr _rawFrame;
        FrameBufferManager<RawFrame>        _rawFrameBuffers;
        FrameBufferManager<DepthFrame>      _depthFrameBuffers;
        FrameBufferManager<PointCloudFrame> _pointCloudBuffers;
        
        FilterSet<RawFrame> _unprocessedFilters, _processedFilters;
        FilterSet<DepthFrame> _depthFilters;
        FrameStreamWriterPtr _frameStreamWriter;
        
        CallbackType _callback[FRAME_TYPE_COUNT];
        PowerChangedCallback _powerChangedCallback;
        
        uint32_t _callBackTypesRegistered = 0;
        ThreadPtr _captureThread = 0;
    };
    typedef Ptr<DepthCamera> DepthCameraPtr;
}

#endif // DEPTHCAMERA_H

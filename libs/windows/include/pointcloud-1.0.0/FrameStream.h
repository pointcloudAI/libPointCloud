/*
 * PointCloud Lib component.
 *
 * Copyright (c) 2018 PointCloud.ai Inc.
 */

#ifndef POINTCLOUD_FRAME_STREAM_H
#define POINTCLOUD_FRAME_STREAM_H

#include <Frame.h>
#include <SerializedObject.h>
#include "DataPacket.h"

namespace PointCloud
{
    
    class CameraSystem;
    class POINTCLOUD_EXPORT FrameGenerator;
    typedef Ptr<FrameGenerator> FrameGeneratorPtr;

    
    struct POINTCLOUD_EXPORT FrameStreamHeader
    {
        char version[2]; // 0 -> major, 1 -> minor
        GeneratorIDType generatorIDs[3]; // For raw (processed), depth and point cloud, in that order
    };
    
    struct POINTCLOUD_EXPORT FrameStreamPacket: public DataPacket
    {
        enum PacketType
        {
            PACKET_DATA = 0,
            PACKET_GENERATOR_CONFIG = 1
        };
        
        FrameStreamPacket(): DataPacket() {}
    };
    
    struct POINTCLOUD_EXPORT GeneratorConfigurationSubPacket
    {
        uint8_t frameType;
        uint32_t size;
        SerializedObject config;
        
        bool read(SerializedObject &object);
        bool write(SerializedObject &object);
    };
    
    class POINTCLOUD_EXPORT FrameStreamWriter
    {
        OutputFileStream &_stream;
        OutputFileStream _internalStream;
        
        Mutex _mutex;
        
        bool _isPaused = false;
        
        size_t _frameCount;
        FrameStreamHeader _header;
        FrameStreamPacket _rawpacket, _configPacket;
        GeneratorConfigurationSubPacket _generatorConfigSubPacket;
        
        bool _writeHeader();
        
        bool _init(GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        
    public:
        FrameStreamWriter(const String &filename, GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        FrameStreamWriter(OutputFileStream &stream, GeneratorIDType processedRawFrameGeneratorID, GeneratorIDType depthFrameGeneratorID, GeneratorIDType pointCloudFrameGeneratorID);
        
        inline bool isStreamGood() { return _stream.good(); }
        
        bool pause();
        bool resume();
        inline bool isPaused() { return _isPaused; }
        
        bool write(FramePtr rawUnprocessed);
        
        inline SerializedObject &getConfigObject() { return _generatorConfigSubPacket.config; }
        bool writeGeneratorConfiguration(uint frameType);
        // Assumes the config sub-packet has been populated by using getConfigObject()
        
        bool close();
        
        virtual ~FrameStreamWriter() { close(); }
        
        
    };
    
    typedef Ptr<FrameStreamWriter> FrameStreamWriterPtr;
    
    class POINTCLOUD_EXPORT FrameStreamReader
    {
        InputFileStream &_stream;
        InputFileStream _internalStream;
        
        Vector<FileOffsetType> _allPacketOffsets;
        
        Vector<IndexType> _dataPacketLocation, _configPacketLocation;
        
        FrameStreamHeader _header;
        
        size_t _currentPacketIndex; // index on _allPacketOffsets
        size_t _currentFrameIndex; // index on _dataPacketLocation
        
        CameraSystem &_sys;
        
        FrameGeneratorPtr _frameGenerator[3]; // for processed raw, depth and point cloud
        
        FrameStreamPacket _dataPacket, _configPacket;
        GeneratorConfigurationSubPacket _configSubPacket;
        
        bool _init();
        
        bool _getPacket(size_t packetIndex, FrameStreamPacket &packet);
        bool _readConfigPacket(size_t packetIndex);
        
    public:
        
        FrameStreamReader(const String &fileName, CameraSystem &sys);
        FrameStreamReader(InputFileStream &stream, CameraSystem &sys);
        
        inline bool isStreamGood() { return _stream.good(); }
        
        Vector<FramePtr> frames; // 4 entries - raw (2 types), depth and point cloud corresponding to currently read frame index
        
        bool readNext();
        bool seekTo(size_t position);
        
        FrameStreamHeader *getFrameHeader()
        {
            return &_header;
        }
         FrameStreamPacket *getFrameConfigPacket(int idx)
        {
            if(idx<0||idx>4)
            {
                return NULL;
            }
            bool ret= _getPacket(idx, _configPacket);
            if(!ret)
            {
                return NULL;
            }
            return &_configPacket;
            
        }
         FrameStreamPacket *getFrameCurrDataPacket()
        {
            return &_dataPacket;
        }
        
        inline size_t currentPosition() { return _currentFrameIndex; }
        inline size_t size() { return _dataPacketLocation.size(); }
        
        template <typename T>
        bool getStreamParam(const String &name, T &value) const;
        
        bool close();
        
        virtual ~FrameStreamReader() {}
    };
    
    typedef Ptr<FrameStreamReader> FrameStreamReaderPtr;
    
    class POINTCLOUD_EXPORT SocketStreamReader
    {
        CameraSystem &_sys;
        FrameGeneratorPtr _frameGenerator[3]; // for processed raw, depth and point cloud
        FrameStreamPacket _dataPacket;
        bool _init();
        uint _id1;
        uint _id2;
        uint _id3;
        SerializedObject object;
        PointCloud::String _config1;
        PointCloud::String _config2;
        PointCloud::String _config3;
    public:
        SocketStreamReader(CameraSystem &sys,uint id1,uint id2,uint id3,char *config1,int len1,char *config2,int len2,char *config3,int len3);
        
        bool readNext(char *ptr,int tsize);
        
        template <typename T>
        bool getStreamParam(const String &name, T &value) const;
        Vector<FramePtr> frames;
        virtual ~SocketStreamReader() {}
    };
    
    typedef Ptr<SocketStreamReader> SocketStreamReaderPtr;
}

#endif

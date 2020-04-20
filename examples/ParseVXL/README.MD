## ParseVXL


## DepthAscii

This C++ example shows how to extraction frame information from a VXL file.

Here are the log for reference:

```
 ./bin/ParseVXL 
Configuration: SDK install path for type 'lib' = /Users/hanyu/libPointCloud/libs/macos/lib//plugin pt.path:/Users/hanyu/libPointCloud/libs/macos/lib/
DEBUG: getUSBDevice vendor:5ac product:259 type:200
ERROR: @@@ Load  VXL File:../examples/ParseVXL/test.vxl
DEBUG: stream state: fail = 0, bad = 0, eof = 0
DEBUG: table @0 = 0, 1
DEBUG: table @1 = 1, 6.12323e-17
DEBUG: table @2 = 1.22465e-16, -1
DEBUG: table @3 = -1, -1.83697e-16
Input VXL is a valid file. Total frames size is 14. 
DEBUG: table @0 = 0, 1
DEBUG: table @1 = 1, 6.12323e-17
DEBUG: table @2 = 1.22465e-16, -1
DEBUG: table @3 = -1, -1.83697e-16
 index:0 phase:3330 amplitude:846 tsensor:35 tillum:31
 index:0 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206543
 index:1 phase:3329 amplitude:843 tsensor:35 tillum:31
 index:1 x:-0.00733802 y:0.0243231 z:1.16969 i:0.205811
 index:2 phase:3329 amplitude:843 tsensor:35 tillum:31
 index:2 x:-0.00733802 y:0.0243231 z:1.16969 i:0.205811
 index:3 phase:3330 amplitude:844 tsensor:35 tillum:31
 index:3 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206055
 index:4 phase:3330 amplitude:844 tsensor:35 tillum:31
 index:4 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206055
 index:5 phase:3328 amplitude:846 tsensor:35 tillum:31
 index:5 x:-0.00733573 y:0.0243155 z:1.16932 i:0.206543
 index:6 phase:3328 amplitude:846 tsensor:35 tillum:31
 index:6 x:-0.00733573 y:0.0243155 z:1.16932 i:0.206543
 index:7 phase:3328 amplitude:844 tsensor:35 tillum:31
 index:7 x:-0.00733573 y:0.0243155 z:1.16932 i:0.206055
 index:8 phase:3328 amplitude:844 tsensor:35 tillum:31
 index:8 x:-0.00733573 y:0.0243155 z:1.16932 i:0.206055
 index:9 phase:3330 amplitude:843 tsensor:35 tillum:31
 index:9 x:-0.00734032 y:0.0243307 z:1.17006 i:0.205811
 index:10 phase:3330 amplitude:843 tsensor:34 tillum:31
 index:10 x:-0.00734032 y:0.0243307 z:1.17006 i:0.205811
 index:11 phase:3330 amplitude:846 tsensor:35 tillum:31
 index:11 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206543
 index:12 phase:3330 amplitude:846 tsensor:35 tillum:31
 index:12 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206543
 index:13 phase:3330 amplitude:846 tsensor:35 tillum:31
 index:13 x:-0.00734032 y:0.0243307 z:1.17006 i:0.206543
DEBUG: Unloading library '/Users/hanyu/libPointCloud/libs/macos/lib//plugin/libOakcdk.dylib'...
DEBUG: Unloading library '/Users/hanyu/libPointCloud/libs/macos/lib//plugin/libSonycdk.dylib'...

```
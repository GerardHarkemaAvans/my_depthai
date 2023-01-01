#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/ImgDetectionsRoboflow.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_imgdetections_roboflow(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<RawImgDetections, RawBuffer, std::shared_ptr<RawImgDetections>> rawImgDetections(m, "RawImgDetections", DOC(dai, RawImgDetections));
    py::class_<ImgDetectionsRoboflow, Buffer, std::shared_ptr<ImgDetectionsRoboflow>> ImgDetectionsRoboflow(m, "ImgDetectionsRoboflow", DOC(dai, ImgDetectionsRoboflow));
    py::class_<ImgDetectionRoboflow> ImgDetectionRoboflow(m, "ImgDetectionRoboflow", DOC(dai, ImgDetectionRoboflow));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Metadata / raw
    ImgDetectionRoboflow
        .def(py::init<>())
        .def_readwrite("label", &ImgDetectionRoboflow::label)
        .def_readwrite("confidence", &ImgDetectionRoboflow::confidence)
        .def_readwrite("xmin", &ImgDetectionRoboflow::xmin)
        .def_readwrite("ymin", &ImgDetectionRoboflow::ymin)
        .def_readwrite("xmax", &ImgDetectionRoboflow::xmax)
        .def_readwrite("ymax", &ImgDetectionRoboflow::ymax)
        ;

    rawImgDetections
        .def(py::init<>())
        .def_readwrite("detections", &RawImgDetections::detections)
        .def_property("ts",
            [](const RawImgDetections& o){
                double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
                return ts;
            },
            [](RawImgDetections& o, double ts){
                o.ts.sec = ts;
                o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
            }
        )
        .def_property("tsDevice",
            [](const RawImgDetections& o){
                double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
                return ts;
            },
            [](RawImgDetections& o, double ts){
                o.tsDevice.sec = ts;
                o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
            }
        )
        .def_readwrite("sequenceNum", &RawImgDetections::sequenceNum)
        ;

    // Message
    ImgDetectionsRoboflow
        .def(py::init<>(), DOC(dai, ImgDetectionsRoboflow, ImgDetectionsRoboflow))
        .def_property("detections", [](ImgDetectionsRoboflow& det) { return &det.detections; }, [](ImgDetectionsRoboflow& det, std::vector<ImgDetectionRoboflow> val) { det.detections = val; }, DOC(dai, ImgDetectionsRoboflow, detections))
        .def("getTimestamp", &ImgDetectionsRoboflow::getTimestamp, DOC(dai, ImgDetectionsRoboflow, getTimestamp))
        .def("getTimestampDevice", &ImgDetectionsRoboflow::getTimestampDevice, DOC(dai, ImgDetectionsRoboflow, getTimestampDevice))
        .def("getSequenceNum", &ImgDetectionsRoboflow::getSequenceNum, DOC(dai, ImgDetectionsRoboflow, getSequenceNum))
        .def("setTimestamp", &ImgDetectionsRoboflow::setTimestamp, DOC(dai, ImgDetectionsRoboflow, setTimestamp))
        .def("setTimestampDevice", &ImgDetectionsRoboflow::setTimestampDevice, DOC(dai, ImgDetectionsRoboflow, setTimestampDevice))
        .def("setSequenceNum", &ImgDetectionsRoboflow::setSequenceNum, DOC(dai, ImgDetectionsRoboflow, setSequenceNum))
        ;


}

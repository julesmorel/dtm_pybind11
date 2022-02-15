#include "../cpp/dtm.h"

#include <pybind11/stl.h>

#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_dtm(py::module &m) {

    py::class_<dtm>(m, "dtm")
    .def(py::init<std::string,std::string,int,double>(), py::arg("_filename"), py::arg("_filenameRect"), py::arg("_minNumberPointsPerLeaf"), py::arg("_minSizeLeaf"))
    .def("polygonize",
         py::overload_cast<int,int,int>( &dtm::polygonize),
         py::arg("nx"),py::arg("ny"),py::arg("nz"))
    .def("display",
         py::overload_cast<>( &dtm::display))
    .def("applyDeformableModel",
         py::overload_cast<int,double>( &dtm::applyDeformableModel),
         py::arg("numberIteration"),py::arg("gamma"))
    .def("exportDTM",
         py::overload_cast<std::string>( &dtm::exportDTM),
         py::arg("filename"));               
}

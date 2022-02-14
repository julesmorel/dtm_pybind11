#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_dtm(py::module &);

namespace mcl {

PYBIND11_MODULE(dtm, m) {
    // Optional docstring
    m.doc() = "DTM library";

    init_dtm(m);
}
}

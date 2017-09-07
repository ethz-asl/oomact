#ifndef H7F1056BE_0386_416C_8B3D_BAA8A6322505
#define H7F1056BE_0386_416C_8B3D_BAA8A6322505

/**
 * Define tools for modules
 */

namespace aslam {
namespace calibration {

std::string normalizeParamName(const char *);

template <typename T>
inline void writeParam(std::ostream & out, const char * name, const T & value) {
  out << ", " << normalizeParamName(name) << "=" << value;
};

inline void writeParam(std::ostream & out, const char * name, bool value) {
  if(value) { out << ", " << normalizeParamName(name); }
};

#define MODULE_WRITE_PARAM(param) writeParam(out, #param, (param));

}
}
#endif /* H7F1056BE_0386_416C_8B3D_BAA8A6322505 */

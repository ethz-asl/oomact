#ifndef H7F1056BE_0386_416C_8B3D_BAA8A6322505
#define H7F1056BE_0386_416C_8B3D_BAA8A6322505

/**
 * Define tools for modules
 */

#define MODULE_WRITE_PARAMETER(param) out << ", " << normalizeName(#param) << "=" << param;
#define MODULE_WRITE_FLAG(param) if(param) { out << ", " << normalizeName(#param); }

#endif /* H7F1056BE_0386_416C_8B3D_BAA8A6322505 */

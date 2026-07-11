#ifndef FLASH_H
#define FLASH_H

// Preserve the original root-facade macros without polluting the parameters
// owner's public contract.
#define PARAM_FILE_NAME "params_config.txt"
#define SaveInt(key, val)   Param_SaveSingle(key, (float)(val))
#define SaveFloat(key, val) Param_SaveSingle(key, (float)(val))

#include "parameters/parameter_store.h"

#endif

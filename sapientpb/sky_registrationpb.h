#ifndef _REGISTER_PB_H_
#define _REGISTER_PB_H_

#include "../sapient/sky_registration.pb.h"

//std::string generateNodeID();


#ifdef __cplusplus
extern "C" {
#endif

int sapient_register(void);

#ifdef __cplusplus
}
#endif



#endif


#include <linux/ioctl.h>

#define VIZZINI_IOC_MAGIC       	'v'

#define VZIOC_GET_REG           	_IOWR(VIZZINI_IOC_MAGIC, 1, int)
#define VZIOC_SET_REG           	_IOWR(VIZZINI_IOC_MAGIC, 2, int)
#define VZIOC_SET_ADDRESS_MATCH 	_IO(VIZZINI_IOC_MAGIC, 3)
#define VZIOC_SET_PRECISE_FLAGS     	_IO(VIZZINI_IOC_MAGIC, 4)
#define VZIOC_TEST_MODE         	_IO(VIZZINI_IOC_MAGIC, 5)
#define VZIOC_LOOPBACK          	_IO(VIZZINI_IOC_MAGIC, 6)

#define VZ_ADDRESS_UNICAST_S        	0
#define VZ_ADDRESS_BROADCAST_S      	8
#define VZ_ADDRESS_MATCH(U, B)          (0x8000000 | ((B) << VZ_ADDRESS_BROADCAST_S) | ((U) << VZ_ADDRESS_UNICAST_S))
#define VZ_ADDRESS_MATCH_DISABLE    	0

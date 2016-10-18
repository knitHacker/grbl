#ifndef _PROGMAN_H_
#define _PROGMAN_H_

#include <stdint.h>

void progman_init(void);
void progman_execute(void);
bool progman_read(uint8_t *dst);

#endif	/* _PROGMAN_H_ */

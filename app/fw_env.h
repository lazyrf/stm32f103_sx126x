#ifndef _FW_ENV_H
#define _FW_ENV_H

#include "config.h"

#if CONFIG_MODULE_FW_ENV

struct env_image {
        uint32_t crc;
        char data[];
};


struct env {
        void            *image;
        uint32_t        *crc;
        char            *data;
};

int fw_env_write(char *name, char *value);
int fw_printenv(void);
char *fw_getenv(char *name);
int fw_getenv_next(char **pos, char **name, char **val);
int fw_env_open(void);
void fw_env_save(void);

#endif /* CONFIG_MODULE_FW_ENV */

#endif /* __FW_ENV_H__ */



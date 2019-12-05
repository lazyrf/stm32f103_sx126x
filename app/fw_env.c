#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "fw_env.h"

#if CONFIG_MODULE_FW_ENV

#include "m_err.h"
#include "bsp_int_flash.h"
#include "bsp_crc.h"

char data[CONFIG_FW_ENV_SIZE];

static struct env envir;

static char default_env[] = {
        "version=" "1" "\0"
        "\0"		/* Termimate struct environment data with 2 NULLs */
};

/*
 * s1 is either a simple 'name', or a 'name=value' pair.
 * s2 is a 'name=value' pair.
 * If the names match, return the value of s2, else NULL.
 */
static char *_envmatch(char *s1, char *s2)
{
        while (*s1 == *s2++)
		if (*s1++ == '=')
			return s2;
	if (*s1 == '\0' && *(s2 - 1) == '=')
		return s2;
        return NULL;
}


int fw_env_write(char *name, char *value)
{
        int len;
        char *env, *nxt;
        char *oldval = NULL;

        for (nxt = env = envir.data; *env; env = nxt + 1) {
                for (nxt = env; *nxt; ++nxt) {
                        if (nxt >= &envir.data[CONFIG_FW_ENV_SIZE]) {
                                printf("\r\nError: Environment not terminated\r\n");
                                return M_ERR_FW_ENV_NOT_TERMINATED;
                        }
                }

                if ((oldval = _envmatch(name, env)) != NULL)
			break;
        }

        if (oldval) {
                if (*++nxt == '\0') {
                        *env = '\0';
                } else {
                        for (;;) {
                                *env = *nxt++;
                                if ((*env == '\0') && (*nxt == '\0'))
                                        break;
                                ++env;
                        }
                }
                *++env = '\0';
        }

        /* Delete env only */
        if (!value || !strlen(value)) {
                goto finish;
        }

        /* Append new definition at the end */
        for (env = envir.data; *env || *(env + 1); ++env);

        if (env > envir.data)
		++env;

        /*
	 * Overflow when:
	 * "name" + "=" + "val" +"\0\0"  > CONFIG_ENV_SIZE - (env-environment)
	 */
        len = strlen(name) + 2;
        /* add '=' for first arg, ' ' for all others */
        len += strlen(value) + 1;

        if (len > (&envir.data[CONFIG_FW_ENV_SIZE] - env)) {
                printf("\r\nError: environment overflow, \"%s\" deleted\r\n", name);
                return M_ERR_FW_ENV_OVERFLOW;
        }

        while ((*env = *name++) != '\0')
                env++;

        *env = '=';

        while ((*++env = *value++) != '\0');

        *++env = '\0';

finish:

#if 0
        *envir.crc = crc_calc((uint32_t *) envir.data, (CONFIG_FW_ENV_SIZE - sizeof(envir.crc)) / sizeof(uint32_t));

        bsp_int_flash_unlock();
        bsp_int_flash_erase(CONFIG_FW_ENV_START_ADDR, 1);
        flash_setting_addr = CONFIG_FW_ENV_START_ADDR;
        bsp_int_flash_write(&flash_setting_addr, (uint32_t *) envir.image, CONFIG_FW_ENV_SIZE / sizeof(uint32_t));
        bsp_int_flash_lock();
#endif

        return M_ERR_OK;
}


void fw_env_save(void)
{
        uint32_t flash_env_addr;

        *envir.crc = bsp_crc_calc((uint32_t *) envir.data, (CONFIG_FW_ENV_SIZE - sizeof(envir.crc)) / sizeof(uint32_t));

        bsp_int_flash_unlock();
        bsp_int_flash_erase(CONFIG_FW_ENV_START_ADDR, 1);
        flash_env_addr = CONFIG_FW_ENV_START_ADDR;
        bsp_int_flash_write(&flash_env_addr, (uint32_t *) envir.image, CONFIG_FW_ENV_SIZE / sizeof(uint32_t));
        bsp_int_flash_lock();
}


int fw_printenv(void)
{
        char *env, *nxt;

        for (env = envir.data; *env; env = nxt + 1) {
                for (nxt = env; *nxt; ++nxt) {
                        if (nxt > &envir.data[CONFIG_FW_ENV_SIZE]) {
                                printf("\r\nError: Environment not terminated\r\n");
                                return M_ERR_FW_ENV_NOT_TERMINATED;
                        }
                }

                printf ("[ENV] %s\r\n", env);
        }

        return M_ERR_OK;
}

char *fw_getenv(char *name)
{
        char *env, *nxt;

        for (env = envir.data; *env; env = nxt + 1) {
                char *val;

                for (nxt = env; *nxt; ++nxt) {
                        if (nxt >= &envir.data[CONFIG_FW_ENV_SIZE]) {
                                printf("Error: Get env, environment not terminated\r\n");
                                return NULL;
                        }
                }
                val = _envmatch(name, env);
                if (!val) {
                        continue;
                }

                return val;
        }

        return NULL;
}

int fw_getenv_next(char **pos, char **name, char **val)
{
        char *env, *nxt;
        char *env_clone, *str;
        int len;

        if (*pos == NULL) {
                env = envir.data;
        } else {
                env = *pos;
        }

        if (!(*env)) {
                return M_ERR_FW_ENV_END;
        }

        for (nxt = env; *nxt; ++nxt) {
                if (nxt > &envir.data[CONFIG_FW_ENV_SIZE]) {
                        printf("\r\nError: Environment not terminated\r\n");
                        return M_ERR_FW_ENV_NOT_TERMINATED;
                }
        }

        env_clone = (char *) malloc(strlen(env));
        strcpy(env_clone, env);

        str = strtok(env_clone, "=");
        if (str == NULL) {
                *name = NULL;
                goto fail;
        } else {
                len = strlen(str) + 1;
                *name = (char *) malloc(len);
                memset(*name, 0, len);
                memcpy(*name, str, len);
                (*name)[len] = '\0';

                str = strtok(NULL, "=");
                if (str == NULL) {
                        *val = NULL;
                        goto fail;
                } else {
                        len = strlen(str) + 1;
                        *val = (char *) malloc(len);
                        memset(*val, 0, len);
                        memcpy(*val, str, len);
                        (*val)[len] = '\0';
                }
        }

        env = nxt + 1;
        *pos = env;

        free(env_clone);

        return M_ERR_OK;

fail:
        if (env_clone != NULL) {
                free(env_clone);
        }

        return M_ERR_FW_ENV_STRING_SPLIT;
}

int fw_env_open(void)
{
        uint32_t crc, crc_ok;
        struct env_image *image;
        uint32_t flash_start_addr;

        envir.image = data;

        image = (void *) data;
        envir.crc = &image->crc;
        envir.data = image->data;

        flash_start_addr = CONFIG_FW_ENV_START_ADDR;
        bsp_int_flash_read(&flash_start_addr, envir.image, CONFIG_FW_ENV_SIZE / 4);

        crc = bsp_crc_calc((uint32_t *) envir.data, (CONFIG_FW_ENV_SIZE - sizeof(envir.crc)) / sizeof(uint32_t));
        crc_ok = (crc == *envir.crc);
        printf("[ENV] crc = 0x%08x, envir.crc = 0x%08x\r\n", crc,  *envir.crc);
        if (!crc_ok) {
                printf("\r\nWarning: Bad env CRC, using default enviroment\r\n");
                memcpy(envir.data, default_env, sizeof(default_env));

                *envir.crc = bsp_crc_calc((uint32_t *) envir.data, (CONFIG_FW_ENV_SIZE - sizeof(envir.crc)) / sizeof(uint32_t));

                bsp_int_flash_unlock();
                bsp_int_flash_erase(CONFIG_FW_ENV_START_ADDR, 1);
                flash_start_addr = CONFIG_FW_ENV_START_ADDR;
                bsp_int_flash_write(&flash_start_addr, (uint32_t *) envir.image, CONFIG_FW_ENV_SIZE / sizeof(uint32_t));
                bsp_int_flash_lock();
        }

        return M_ERR_OK;
}

#endif /* CONFIG_MODULE_FW_ENV */



#ifndef _M_ERR_H
#define _M_ERR_H

typedef enum {
	M_ERR_OK = 0,
	M_ERR_FAIL = -1,
	M_ERR_PARAM = -2,
	M_ERR_MEM = -3,
	M_ERR_TIMEOUT = -4,
	M_ERR_BUSY = -5,
} m_err_t;

#endif /* _M_ERR_H */

/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/kernel.h>

extern int32_t z_vrfy_k_sleep(k_timeout_t timeout);
uintptr_t z_mrsh_k_sleep(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg2;	/* unused */
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { struct { uintptr_t lo, hi; } split; k_timeout_t val; } parm0;
	parm0.split.lo = arg0;
	parm0.split.hi = arg1;
	int32_t ret = z_vrfy_k_sleep(parm0.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}


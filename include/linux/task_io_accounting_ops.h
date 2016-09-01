/*
 * Task I/O accounting operations
 */
#ifndef __TASK_IO_ACCOUNTING_OPS_INCLUDED
#define __TASK_IO_ACCOUNTING_OPS_INCLUDED

#include <linux/sched.h>
#include <trace/events/mmc.h>

#ifdef CONFIG_TASK_IO_ACCOUNTING
static inline void task_io_account_read(size_t bytes)
{
	current->ioac.read_bytes += bytes;
	if (current->ioac.read_bytes - current->ioac.last_read_bytes > 1048576) {
		trace_mmc_pid_blk_read_summary(current);
//		trace_printk("process [%u] do read io %llu bytes\n", current->pid, current->ioac.read_bytes);
		current->ioac.last_read_bytes = current->ioac.read_bytes;
	}
}

/*
 * We approximate number of blocks, because we account bytes only.
 * A 'block' is 512 bytes
 */
static inline unsigned long task_io_get_inblock(const struct task_struct *p)
{
	return p->ioac.read_bytes >> 9;
}

static inline void task_io_account_write(size_t bytes)
{
	current->ioac.write_bytes += bytes;
	if (current->ioac.write_bytes - current->ioac.last_write_bytes > 1048576) {
		trace_mmc_pid_blk_write_summary(current);
//		trace_printk("process [%u] do write io %llu bytes\n", current->pid, current->ioac.write_bytes);
		current->ioac.last_write_bytes = current->ioac.write_bytes;
	}
}

/*
 * We approximate number of blocks, because we account bytes only.
 * A 'block' is 512 bytes
 */
static inline unsigned long task_io_get_oublock(const struct task_struct *p)
{
	return p->ioac.write_bytes >> 9;
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
	current->ioac.cancelled_write_bytes += bytes;
}

static inline void task_io_accounting_init(struct task_io_accounting *ioac)
{
	memset(ioac, 0, sizeof(*ioac));
}

static inline void task_blk_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	dst->read_bytes += src->read_bytes;
	dst->write_bytes += src->write_bytes;
	dst->cancelled_write_bytes += src->cancelled_write_bytes;

	if (current->ioac.read_bytes - current->ioac.last_read_bytes > 1024*1024) {
		trace_mmc_pid_blk_read_summary(current);
//		trace_printk("process [%u] do read io %llu bytes\n", current->pid, current->ioac.read_bytes);
		current->ioac.last_read_bytes = current->ioac.read_bytes;
	}
	
	if (current->ioac.write_bytes - current->ioac.last_write_bytes > 256*1024) {
		trace_mmc_pid_blk_write_summary(current);
//		trace_printk("process [%u] do write io %llu bytes\n", current->pid, current->ioac.write_bytes);
		current->ioac.last_write_bytes = current->ioac.write_bytes;
	}
}

#else

static inline void task_io_account_read(size_t bytes)
{
}

static inline unsigned long task_io_get_inblock(const struct task_struct *p)
{
	return 0;
}

static inline void task_io_account_write(size_t bytes)
{
}

static inline unsigned long task_io_get_oublock(const struct task_struct *p)
{
	return 0;
}

static inline void task_io_account_cancelled_write(size_t bytes)
{
}

static inline void task_io_accounting_init(struct task_io_accounting *ioac)
{
}

static inline void task_blk_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
}

#endif /* CONFIG_TASK_IO_ACCOUNTING */

#ifdef CONFIG_TASK_XACCT
static inline void task_chr_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	dst->rchar += src->rchar;
	dst->wchar += src->wchar;
	dst->syscr += src->syscr;
	dst->syscw += src->syscw;
}
#else
static inline void task_chr_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
}
#endif /* CONFIG_TASK_XACCT */

static inline void task_io_accounting_add(struct task_io_accounting *dst,
						struct task_io_accounting *src)
{
	task_chr_io_accounting_add(dst, src);
	task_blk_io_accounting_add(dst, src);
}
#endif /* __TASK_IO_ACCOUNTING_OPS_INCLUDED */

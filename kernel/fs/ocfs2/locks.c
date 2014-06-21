

#include <linux/fs.h>
#include <linux/fcntl.h>

#define MLOG_MASK_PREFIX ML_INODE
#include <cluster/masklog.h>

#include "ocfs2.h"

#include "dlmglue.h"
#include "file.h"
#include "inode.h"
#include "locks.h"

static int ocfs2_do_flock(struct file *file, struct inode *inode,
			  int cmd, struct file_lock *fl)
{
	int ret = 0, level = 0, trylock = 0;
	struct ocfs2_file_private *fp = file->private_data;
	struct ocfs2_lock_res *lockres = &fp->fp_flock;

	if (fl->fl_type == F_WRLCK)
		level = 1;
	if (!IS_SETLKW(cmd))
		trylock = 1;

	mutex_lock(&fp->fp_mutex);

	if (lockres->l_flags & OCFS2_LOCK_ATTACHED &&
	    lockres->l_level > LKM_NLMODE) {
		int old_level = 0;

		if (lockres->l_level == LKM_EXMODE)
			old_level = 1;

		if (level == old_level)
			goto out;

		/*
		 * Converting an existing lock is not guaranteed to be
		 * atomic, so we can get away with simply unlocking
		 * here and allowing the lock code to try at the new
		 * level.
		 */

		flock_lock_file_wait(file,
				     &(struct file_lock){.fl_type = F_UNLCK});

		ocfs2_file_unlock(file);
	}

	ret = ocfs2_file_lock(file, level, trylock);
	if (ret) {
		if (ret == -EAGAIN && trylock)
			ret = -EWOULDBLOCK;
		else
			mlog_errno(ret);
		goto out;
	}

	ret = flock_lock_file_wait(file, fl);

out:
	mutex_unlock(&fp->fp_mutex);

	return ret;
}

static int ocfs2_do_funlock(struct file *file, int cmd, struct file_lock *fl)
{
	int ret;
	struct ocfs2_file_private *fp = file->private_data;

	mutex_lock(&fp->fp_mutex);
	ocfs2_file_unlock(file);
	ret = flock_lock_file_wait(file, fl);
	mutex_unlock(&fp->fp_mutex);

	return ret;
}

int ocfs2_flock(struct file *file, int cmd, struct file_lock *fl)
{
	struct inode *inode = file->f_mapping->host;
	struct ocfs2_super *osb = OCFS2_SB(inode->i_sb);

	if (!(fl->fl_flags & FL_FLOCK))
		return -ENOLCK;
	if (__mandatory_lock(inode))
		return -ENOLCK;

	if ((osb->s_mount_opt & OCFS2_MOUNT_LOCALFLOCKS) ||
	    ocfs2_mount_local(osb))
		return flock_lock_file_wait(file, fl);

	if (fl->fl_type == F_UNLCK)
		return ocfs2_do_funlock(file, cmd, fl);
	else
		return ocfs2_do_flock(file, inode, cmd, fl);
}

int ocfs2_lock(struct file *file, int cmd, struct file_lock *fl)
{
	struct inode *inode = file->f_mapping->host;
	struct ocfs2_super *osb = OCFS2_SB(inode->i_sb);

	if (!(fl->fl_flags & FL_POSIX))
		return -ENOLCK;
	if (__mandatory_lock(inode) && fl->fl_type != F_UNLCK)
		return -ENOLCK;

	return ocfs2_plock(osb->cconn, OCFS2_I(inode)->ip_blkno, file, cmd, fl);
}

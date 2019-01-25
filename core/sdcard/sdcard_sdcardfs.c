/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 * Copyright 2015 spreatrum
 * modify by neil wang for androidm sdcardfs
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "sdcard"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <linux/fuse.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/param.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cutils/fs.h>
#include <cutils/hashmap.h>
#include <cutils/log.h>
#include <cutils/multiuser.h>

#include <private/android_filesystem_config.h>

/* README
 *
 * What is this?
 *
 * sdcard is a program that uses sdcardfs to emulate FAT-on-sdcard style
 * directory permissions (all files are given fixed owner, group, and
 * permissions at creation, owner, group, and permissions are not
 * changeable, symlinks and hardlinks are not createable, etc.
 *
 * See usage() for command line options.
 *
 * It must be run as root, but will drop to requested UID/GID as soon as it
 * mounts a filesystem.	 It will refuse to run if requested UID/GID are zero.
 *
 * Things I believe to be true:
 *
 * - ops that return a fuse_entry (LOOKUP, MKNOD, MKDIR, LINK, SYMLINK,
 * CREAT) must bump that node's refcount
 * - don't forget that FORGET can forget multiple references (req->nlookup)
 * - if an op that returns a fuse_entry fails writing the reply to the
 * kernel, you must rollback the refcount to reflect the reference the
 * kernel did not actually acquire
 *
 * This daemon can also derive custom filesystem permissions based on directory
 * structure when requested. These custom permissions support several features:
 *
 * - Apps can access their own files in /Android/data/com.example/ without
 * requiring any additional GIDs.
 * - Separate permissions for protecting directories like Pictures and Music.
 * - Multi-user separation on the same physical device.
 */

#define FUSE_TRACE 0

#if FUSE_TRACE
#define TRACE(x...) ALOGD(x)
#else
#define TRACE(x...) do {} while (0)
#endif

#define ERROR(x...) ALOGE(x)

/* Supplementary groups to execute with */
static const gid_t kGroups[1] = { AID_PACKAGE_INFO };

typedef struct sdcardfs_mount_option {
    char dest_path[PATH_MAX];
    gid_t gid;
    mode_t mask;
} sdcardfs_mount_type;

static int usage() {
    ERROR("usage: sdcard [OPTIONS] <source_path> <label>\n"
	    "	 -u: specify UID to run as\n"
	    "	 -g: specify GID to run as\n"
	    "	 -U: specify user ID that owns device\n"
	    "	 -m: source_path is multi-user\n"
	    "	 -w: runtime write mount has full write access\n"
	    "\n");
    return 1;
}

static int sdcardfs_setup(const char* source_path, const char* dest_path, char* opts) {

    umount2(dest_path, MNT_DETACH);

    if (mount(source_path, dest_path, "sdcardfs", MS_NOSUID | MS_NODEV | MS_NOEXEC |
	    MS_NOATIME, opts) != 0) {
	ERROR("failed to mount sdcardfs filesystem: %s\n", strerror(errno));
	return -1;
    }

    return 0;
}

static void run(const char* source_path, const char* label, uid_t uid,
	gid_t gid, userid_t userid, bool multi_user, bool full_write) {

    sdcardfs_mount_type sdcardfs_type[3];
    char opts[256];
    int i = 0;

    snprintf(sdcardfs_type[0].dest_path, PATH_MAX, "/mnt/runtime/default/%s", label);
    snprintf(sdcardfs_type[1].dest_path, PATH_MAX, "/mnt/runtime/read/%s", label);
    snprintf(sdcardfs_type[2].dest_path, PATH_MAX, "/mnt/runtime/write/%s", label);

    umask(0);

    /* careful taken should come first */
    for (i = 0;i < 3; i++) {
	snprintf(opts, sizeof(opts),
	       "token=%d,uid=%d,gid=%d,label=%s,multi_user=%d,full_write=%d",
		 i,uid, gid, label, multi_user,full_write);

	if (sdcardfs_setup(source_path, sdcardfs_type[i].dest_path, opts))
	    exit(1);
    }

    /* Drop privs */
    if (setgroups(sizeof(kGroups) / sizeof(kGroups[0]), kGroups) < 0) {
	ERROR("cannot setgroups: %s\n", strerror(errno));
	exit(1);
    }
    if (setgid(gid) < 0) {
	ERROR("cannot setgid: %s\n", strerror(errno));
	exit(1);
    }
    if (setuid(uid) < 0) {
	ERROR("cannot setuid: %s\n", strerror(errno));
	exit(1);
    }

    /* just sleeping */
    while (1) {
	sleep(100);
    }

    ERROR("terminated prematurely\n");
    exit(1);
}

int main(int argc, char **argv) {
    const char *source_path = NULL;
    const char *label = NULL;
    uid_t uid = 0;
    gid_t gid = 0;
    userid_t userid = 0;
    bool multi_user = false;
    bool full_write = false;
    int i;
    struct rlimit rlim;
    int fs_version;

    int opt;
    while ((opt = getopt(argc, argv, "u:g:U:mw")) != -1) {
	switch (opt) {
	    case 'u':
		uid = strtoul(optarg, NULL, 10);
		break;
	    case 'g':
		gid = strtoul(optarg, NULL, 10);
		break;
	    case 'U':
		userid = strtoul(optarg, NULL, 10);
		break;
	    case 'm':
		multi_user = true;
		break;
	    case 'w':
		full_write = true;
		break;
	    case '?':
	    default:
		return usage();
	}
    }

    for (i = optind; i < argc; i++) {
	char* arg = argv[i];
	if (!source_path) {
	    source_path = arg;
	} else if (!label) {
	    label = arg;
	} else {
	    ERROR("too many arguments\n");
	    return usage();
	}
    }

    if (!source_path) {
	ERROR("no source path specified\n");
	return usage();
    }
    if (!label) {
	ERROR("no label specified\n");
	return usage();
    }
    if (!uid || !gid) {
	ERROR("uid and gid must be nonzero\n");
	return usage();
    }

    rlim.rlim_cur = 8192;
    rlim.rlim_max = 8192;
    if (setrlimit(RLIMIT_NOFILE, &rlim)) {
	ERROR("Error setting RLIMIT_NOFILE, errno = %d\n", errno);
    }
#if 0 //merge_kaios
    while ((fs_read_atomic_int("/data/.layout_version", &fs_version) == -1) || (fs_version < 3)) {
	ERROR("installd fs upgrade not yet complete. Waiting...\n");
	sleep(1);
    }
#endif
    run(source_path, label, uid, gid, userid, multi_user, full_write);
    return 1;
}

/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "fs/Vfat.h"
#include "fs/Exfat.h"
#include "PublicVolume.h"
#include "Utils.h"
#include "VolumeManager.h"
#include "ResponseCode.h"

#include <base/stringprintf.h>
#include <base/logging.h>
#include <cutils/fs.h>
/* SPRD: add for physical internal SD */
#include <cutils/properties.h>
#include <private/android_filesystem_config.h>

#include <fcntl.h>
#include <stdlib.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>

using android::base::StringPrintf;

namespace android {
namespace vold {

#define CARD_SIZE_32G (32*1024*1024*1024ULL)

static const char* kFusePath = "/system/bin/sdcard";

static const char* kAsecPath = "/mnt/secure/asec";

/* SPRD: Add support for install apk to internal sdcard @{*/
static const char* kInternalSdAsecPath = "/mnt/secure/internal-asec";
/* @} */

PublicVolume::PublicVolume(dev_t device) :
        VolumeBase(Type::kPublic), mDevice(device), mFusePid(0) {
    setId(StringPrintf("public:%u,%u", major(device), minor(device)));
    mDevPath = StringPrintf("/dev/block/vold/%s", getId().c_str());
}

PublicVolume::~PublicVolume() {
}

status_t PublicVolume::readMetadata() {
    status_t res = ReadMetadataUntrusted(mDevPath, mFsType, mFsUuid, mFsLabel);
    /* SPRD: add for physical internal SD @{ */
    if (!VolumeManager::isInternalEmulated()
            && getLinkName() == "sdcard0") {
        // this is physical internal SD
        LOG(VERBOSE) << StringPrintf("readMetadata()=%d, mFsType is %s, mFsUuid is %s, mFsLabel is %s",
                                 res, mFsType.c_str(), mFsUuid.c_str(), mFsLabel.c_str());
        if (res < 0) {
            if (vfat::Format(mDevPath, 0)) {
                LOG(ERROR) << getId() << " failed to format";
            } else {
                res = ReadMetadataUntrusted(mDevPath, mFsType, mFsUuid, mFsLabel);
                LOG(VERBOSE) << StringPrintf("readMetadata()=%d, mFsType is %s, mFsUuid is %s, mFsLabel is %s",
                                             res, mFsType.c_str(), mFsUuid.c_str(), mFsLabel.c_str());
            }
        }
        if (mFsLabel == "") {
            char value[PROPERTY_VALUE_MAX];
            property_get("ro.internal.physical.name", value, "Spreadtrum");
            mFsLabel = value;
            LOG(VERBOSE) << StringPrintf("set physical internal mFsLabel as %s", value);
        }
    }
    /* @} */
    notifyEvent(ResponseCode::VolumeFsTypeChanged, mFsType);
    notifyEvent(ResponseCode::VolumeFsUuidChanged, mFsUuid);
    notifyEvent(ResponseCode::VolumeFsLabelChanged, mFsLabel);
    return res;
}

status_t PublicVolume::initAsecStage() {
    std::string legacyPath(mRawPath + "/android_secure");
    std::string securePath(mRawPath + "/.android_secure");

    // Recover legacy secure path
    if (!access(legacyPath.c_str(), R_OK | X_OK)
            && access(securePath.c_str(), R_OK | X_OK)) {
        if (rename(legacyPath.c_str(), securePath.c_str())) {
            PLOG(WARNING) << getId() << " failed to rename legacy ASEC dir";
        }
    }

    if (TEMP_FAILURE_RETRY(mkdir(securePath.c_str(), 0700))) {
        if (errno != EEXIST) {
            PLOG(WARNING) << getId() << " creating ASEC stage failed";
            return -errno;
        }
    }
    /* SPRD: Add support for install apk to internal sdcard @{
     * @orig    BindMount(securePath, kAsecPath);
     */
    if(getLinkName() == "sdcard0" || getLinkName() == "sdcard1"){
    if (!VolumeManager::isInternalEmulated()
            && getLinkName() == "sdcard0") {
    	BindMount(securePath, kInternalSdAsecPath);
    }else{
        BindMount(securePath, kAsecPath);
    }
    }
    /* @ */

    return OK;
}

status_t PublicVolume::doCreate() {
    return CreateDeviceNode(mDevPath, mDevice);
}

status_t PublicVolume::doDestroy() {
    /* SPRD: add for storage */
    //property_set(StringPrintf("vold.%s.path", getLinkName().c_str()).c_str(), "");
    return DestroyDeviceNode(mDevPath);
}

/* SPRD: add for read storage metadata */
status_t PublicVolume::doGetMetadata() {
    readMetadata();
    return OK;
}
/* @} */

status_t PublicVolume::doMount() {
    // TODO: expand to support mounting other filesystems
    readMetadata();

    if (mFsType == "exfat" && exfat::IsSupported()) {
        LOG(VERBOSE) << getId() << " detects filesystem " << mFsType;
    } else if (mFsType != "vfat") {
        LOG(ERROR) << getId() << " unsupported filesystem " << mFsType;
        return -EIO;
    }

    if (mFsType == "vfat") {
        if (vfat::Check(mDevPath)) {
            LOG(ERROR) << getId() << " failed filesystem check";
            return -EIO;
        }
    } else if (mFsType == "exfat") {
        if (exfat::Check(mDevPath)) {
            LOG(ERROR) << getId() << " failed filesystem check";
            return -EIO;
        }
    }

    // Use UUID as stable name, if available
    std::string stableName = getId();
    if (!mFsUuid.empty()) {
        stableName = mFsUuid;
    }

    mRawPath = StringPrintf("/mnt/media_rw/%s", stableName.c_str());

    mFuseDefault = StringPrintf("/mnt/runtime/default/%s", stableName.c_str());
    mFuseRead = StringPrintf("/mnt/runtime/read/%s", stableName.c_str());
    mFuseWrite = StringPrintf("/mnt/runtime/write/%s", stableName.c_str());

    setInternalPath(mRawPath);
    if (getMountFlags() & MountFlags::kVisible) {
        setPath(StringPrintf("/storage/%s", stableName.c_str()));
    } else {
        setPath(mRawPath);
    }

    if (fs_prepare_dir(mRawPath.c_str(), 0700, AID_ROOT, AID_ROOT) ||
            fs_prepare_dir(mFuseDefault.c_str(), 0700, AID_ROOT, AID_ROOT) ||
            fs_prepare_dir(mFuseRead.c_str(), 0700, AID_ROOT, AID_ROOT) ||
            fs_prepare_dir(mFuseWrite.c_str(), 0700, AID_ROOT, AID_ROOT)) {
        PLOG(ERROR) << getId() << " failed to create mount points";
        return -errno;
    }

    /* SPRD: add for storage, create link for fuse path @{ */
    if (!getLinkName().empty()) {
        LOG(VERBOSE) << "create link for fuse path, linkName=" << getLinkName();
        CreateSymlink(stableName, StringPrintf("/mnt/runtime/default/%s", getLinkName().c_str()));
        CreateSymlink(stableName, StringPrintf("/mnt/runtime/read/%s", getLinkName().c_str()));
        CreateSymlink(stableName, StringPrintf("/mnt/runtime/write/%s", getLinkName().c_str()));
        property_set(StringPrintf("vold.%s.path", getLinkName().c_str()).c_str(),
                StringPrintf("/storage/%s", stableName.c_str()).c_str());
    }
    /* @} */

    if (mFsType == "vfat") {
        if (vfat::Mount(mDevPath, mRawPath, false, false, false,
                AID_MEDIA_RW, AID_MEDIA_RW, 0007, true)) {
            PLOG(ERROR) << getId() << " failed to mount " << mDevPath;
            return -EIO;
        }
    } else if (mFsType == "exfat") {
        if (exfat::Mount(mDevPath, mRawPath, false, false, false,
                AID_MEDIA_RW, AID_MEDIA_RW, 0007, true)) {
            PLOG(ERROR) << getId() << " failed to mount " << mDevPath;
            return -EIO;
        }
    }

   /* SPRD: Add support for install apk to internal sdcard @{
    * @orig if (getMountFlags() & MountFlags::kPrimary) {
    *         initAsecStage();
    * }
  */
    initAsecStage();
    /* @} */
    if (!(getMountFlags() & MountFlags::kVisible)) {
        // Not visible to apps, so no need to spin up FUSE
        return OK;
    }

    dev_t before = GetDevice(mFuseWrite);

    if (!(mFusePid = fork())) {
        if (getMountFlags() & MountFlags::kPrimary) {
            if (execl(kFusePath, kFusePath,
                    "-u", "1023", // AID_MEDIA_RW
                    "-g", "1023", // AID_MEDIA_RW
                    "-U", std::to_string(getMountUserId()).c_str(),
                    "-w",
                    mRawPath.c_str(),
                    stableName.c_str(),
                    NULL)) {
                PLOG(ERROR) << "Failed to exec";
            }
        } else {
            if (execl(kFusePath, kFusePath,
                    "-u", "1023", // AID_MEDIA_RW
                    "-g", "1023", // AID_MEDIA_RW
                    "-U", std::to_string(getMountUserId()).c_str(),
                    // SPRD: add for not primary volume writable
                    "-w",
                    mRawPath.c_str(),
                    stableName.c_str(),
                    NULL)) {
                PLOG(ERROR) << "Failed to exec";
            }
        }

        LOG(ERROR) << "FUSE exiting";
        _exit(1);
    }

    if (mFusePid == -1) {
        PLOG(ERROR) << getId() << " failed to fork";
        return -errno;
    }

    while (before == GetDevice(mFuseWrite)) {
        LOG(VERBOSE) << "Waiting for FUSE to spin up...";
        usleep(50000); // 50ms
    }

    return OK;
}

status_t PublicVolume::doUnmount() {
    if (mFusePid > 0) {
        kill(mFusePid, SIGTERM);
        TEMP_FAILURE_RETRY(waitpid(mFusePid, nullptr, 0));
        mFusePid = 0;
    }

    /* SPRD: Add support for install apk to internal sdcard @{
     *@orig  ForceUnmount(kAsecPath);
    */
    if(getLinkName() == "sdcard0" || getLinkName() == "sdcard1"){
    if (!VolumeManager::isInternalEmulated()
            && getLinkName() == "sdcard0") {
         ForceUnmount(kInternalSdAsecPath);
    }else{
         ForceUnmount(kAsecPath);
    }
    }
     /* @ */
    /* SPRD: add for storage, delete link for fuse path @{
     *  */
    if (!getLinkName().empty()) {
        LOG(VERBOSE) << "delete link for fuse path, linkName=" << getLinkName();
        DeleteSymlink(StringPrintf("/mnt/runtime/default/%s", getLinkName().c_str()));
        DeleteSymlink(StringPrintf("/mnt/runtime/read/%s", getLinkName().c_str()));
        DeleteSymlink(StringPrintf("/mnt/runtime/write/%s", getLinkName().c_str()));
    }
    /* @* } */

    ForceUnmount(mFuseDefault);
    ForceUnmount(mFuseRead);
    ForceUnmount(mFuseWrite);
    ForceUnmount(mRawPath);


    rmdir(mFuseDefault.c_str());
    rmdir(mFuseRead.c_str());
    rmdir(mFuseWrite.c_str());
    rmdir(mRawPath.c_str());

    mFuseDefault.clear();
    mFuseRead.clear();
    mFuseWrite.clear();
    mRawPath.clear();

    return OK;
}

/* SPRD: add for UMS @{ */
status_t PublicVolume::doShare(const std::string& massStorageFilePath) {
    mMassStorageFilePath = massStorageFilePath;

    std::string shareDevPath;
    VolumeManager *vm = VolumeManager::Instance();
    auto disk = vm->findDisk(getDiskId());
    std::string partname = disk->getPartname();
    if (partname.empty()) {
        // external physical SD card, share whole disk
        shareDevPath = disk->getDevPath();
    } else {
        // internal SD, just share a partition
        shareDevPath = mDevPath;
    }

    return android::vold::WriteToFile(getId(), mMassStorageFilePath, shareDevPath, 0);
}

status_t PublicVolume::doUnshare() {
    if (mMassStorageFilePath.empty()) {
        LOG(WARNING) << "mass storage file path is empty";
        return -1;
    }
    return android::vold::WriteToFile(getId(), mMassStorageFilePath, std::string(), 0);
}

status_t PublicVolume::doSetState(State state) {
    if (getLinkName().empty()) {
        LOG(WARNING) << "LinkName is empty, this is not a physical storage.";
    } else {
        property_set(StringPrintf("vold.%s.state", getLinkName().c_str()).c_str(), findState(state).c_str());
    }
    return OK;
}
/* @} */

status_t PublicVolume::doFormat(const std::string& fsType) {
    int  fsVfat = 0, fsExfat = 0;
    unsigned long long size64 = 0ull;

    if (getBlkDeviceSize(mDevPath, size64) != OK) {
        LOG(ERROR) << getId() << " failed to get size of block device.";
    }

    if ((fsType == "exfat" || (fsType == "auto" && size64 > CARD_SIZE_32G)) && exfat::IsSupported()) {
        fsExfat = 1;
        LOG(VERBOSE) << "Format to " << fsType << " for device block size " << size64;
    } else if (fsType == "vfat"  || fsType == "auto") {
        fsVfat = 1;
    } else {
        LOG(ERROR) << "Unsupported filesystem " << fsType;
        return -EINVAL;
    }

    if (WipeBlockDevice(mDevPath) != OK) {
        LOG(WARNING) << getId() << " failed to wipe";
    }

    if (fsVfat) {
        if (vfat::Format(mDevPath, 0)) {
            LOG(ERROR) << getId() << " failed to format(vfat)";
            return -errno;
        }
    } else {
        if (exfat::Format(mDevPath, 0)) {
            LOG(ERROR) << getId() << " failed to format(exfat)";
            return -errno;
        }
    }

    return OK;
}

}  // namespace vold
}  // namespace android

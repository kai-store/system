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

#include "Utils.h"
#include "VolumeBase.h"
#include "VolumeManager.h"
#include "ResponseCode.h"

#include <base/stringprintf.h>
#include <base/logging.h>

#include <fcntl.h>
#include <stdlib.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>

using android::base::StringPrintf;

#define DEBUG 1

namespace android {
namespace vold {

VolumeBase::VolumeBase(Type type) :
        mType(type), mMountFlags(0), mMountUserId(-1), mCreated(false), mState(
                State::kUnmounted), mSilent(false) {
}

VolumeBase::~VolumeBase() {
    CHECK(!mCreated);
}

void VolumeBase::setState(State state) {
    mState = state;
    /* SPRD: add for storage */
    doSetState(state);
    notifyEvent(ResponseCode::VolumeStateChanged, StringPrintf("%d", mState));
}

status_t VolumeBase::setDiskId(const std::string& diskId) {
    if (mCreated) {
        LOG(WARNING) << getId() << " diskId change requires destroyed";
        return -EBUSY;
    }

    mDiskId = diskId;
    return OK;
}

status_t VolumeBase::setPartGuid(const std::string& partGuid) {
    if (mCreated) {
        LOG(WARNING) << getId() << " partGuid change requires destroyed";
        return -EBUSY;
    }

    mPartGuid = partGuid;
    return OK;
}

status_t VolumeBase::setMountFlags(int mountFlags) {
    if ((mState != State::kUnmounted) && (mState != State::kUnmountable)) {
        LOG(WARNING) << getId() << " flags change requires state unmounted or unmountable";
        return -EBUSY;
    }

    mMountFlags = mountFlags;
    return OK;
}

status_t VolumeBase::setMountUserId(userid_t mountUserId) {
    if ((mState != State::kUnmounted) && (mState != State::kUnmountable)) {
        LOG(WARNING) << getId() << " user change requires state unmounted or unmountable";
        return -EBUSY;
    }

    mMountUserId = mountUserId;
    return OK;
}

status_t VolumeBase::setSilent(bool silent) {
    if (mCreated) {
        LOG(WARNING) << getId() << " silence change requires destroyed";
        return -EBUSY;
    }

    mSilent = silent;
    return OK;
}

/* SPRD: set link name for mount path @{ */
status_t VolumeBase::setLinkName(const std::string& linkName) {
    if (mCreated) {
        LOG(WARNING) << getId() << " linkName change requires destroyed";
        return -EBUSY;
    }

    mLinkname = linkName;
    return OK;
}
/* @} */

status_t VolumeBase::setId(const std::string& id) {
    if (mCreated) {
        LOG(WARNING) << getId() << " id change requires not created";
        return -EBUSY;
    }

    mId = id;
    return OK;
}

status_t VolumeBase::setPath(const std::string& path) {
    if (mState != State::kChecking) {
        LOG(WARNING) << getId() << " path change requires state checking";
        return -EBUSY;
    }

    mPath = path;
    notifyEvent(ResponseCode::VolumePathChanged, mPath);
    return OK;
}

status_t VolumeBase::setInternalPath(const std::string& internalPath) {
    if (mState != State::kChecking) {
        LOG(WARNING) << getId() << " internal path change requires state checking";
        return -EBUSY;
    }

    mInternalPath = internalPath;
    notifyEvent(ResponseCode::VolumeInternalPathChanged, mInternalPath);
    return OK;
}

void VolumeBase::notifyEvent(int event) {
    if (mSilent) return;
    VolumeManager::Instance()->getBroadcaster()->sendBroadcast(event,
            getId().c_str(), false);
}

void VolumeBase::notifyEvent(int event, const std::string& value) {
    if (mSilent) return;
    VolumeManager::Instance()->getBroadcaster()->sendBroadcast(event,
            StringPrintf("%s %s", getId().c_str(), value.c_str()).c_str(), false);
}

void VolumeBase::addVolume(const std::shared_ptr<VolumeBase>& volume) {
    mVolumes.push_back(volume);
}

void VolumeBase::removeVolume(const std::shared_ptr<VolumeBase>& volume) {
    mVolumes.remove(volume);
}

std::shared_ptr<VolumeBase> VolumeBase::findVolume(const std::string& id) {
    for (auto vol : mVolumes) {
        if (vol->getId() == id) {
            return vol;
        }
    }
    return nullptr;
}

status_t VolumeBase::create() {
    CHECK(!mCreated);

    mCreated = true;
    status_t res = doCreate();
    /* SPRD: add this to fix a problem */
    mState = State::kUnmounted;
    /* SPRD: modify for storage manage @{
    notifyEvent(ResponseCode::VolumeCreated,
            StringPrintf("%d \"%s\" \"%s\"", mType, mDiskId.c_str(), mPartGuid.c_str()));
     */
    notifyEvent(ResponseCode::VolumeCreated,
            StringPrintf("%d \"%s\" \"%s\" \"%s\"", mType, mDiskId.c_str(), mPartGuid.c_str(), mLinkname.c_str()));
    /* @} */
    /* SPRD: add for bug 552546 @{
     * @org setState(State::kUnmounted);
    */
    if (mState != State::kChecking) {
        setState(State::kUnmounted);
    }
    /* @} */
    /* SPRD: add for read storage metadata @{ */
    if (res == OK) {
        res = getMetadata();
    }
    /* @} */
    return res;
}

/* SPRD: add for read storage metadata @{ */
status_t VolumeBase::getMetadata() {
    CHECK(mCreated);

    status_t res = doGetMetadata();
    return res;
}
status_t VolumeBase::doGetMetadata() {
    return OK;
}
/* @} */

status_t VolumeBase::doCreate() {
    return OK;
}

status_t VolumeBase::destroy() {
    CHECK(mCreated);

    if (mState == State::kMounted) {
        unmount();
        setState(State::kBadRemoval);
    /* SPRD: add for UMS @{ */
    } else if (mState == State::kShared) {
            doUnshare();
            LOG(WARNING) << "The state is shared ,need to be unshared";
            VolumeManager *vm = VolumeManager::Instance();
            vm->mUmsSharedCount = 0;
            vm->mUmsShareIndex = -1;
            vm->mUmsSharePrepareCount = 0;
            setState(State::kBadRemoval);
    /* @} */
        } else {
            setState(State::kRemoved);
        }

    notifyEvent(ResponseCode::VolumeDestroyed);
    status_t res = doDestroy();
    mCreated = false;
    return res;
}

status_t VolumeBase::doDestroy() {
    return OK;
}

status_t VolumeBase::mount() {
    if ((mState != State::kUnmounted) && (mState != State::kUnmountable)) {
        LOG(WARNING) << getId() << " mount requires state unmounted or unmountable";
        return -EBUSY;
    }

    setState(State::kChecking);
    status_t res = doMount();
    if (res == OK) {
        setState(State::kMounted);
    } else {
        setState(State::kUnmountable);
    }

    return res;
}

status_t VolumeBase::unmount() {
    if (mState != State::kMounted) {
        LOG(WARNING) << getId() << " unmount requires state mounted";
        return -EBUSY;
    }

    setState(State::kEjecting);
    for (auto vol : mVolumes) {
        if (vol->destroy()) {
            LOG(WARNING) << getId() << " failed to destroy " << vol->getId()
                    << " stacked above";
        }
    }
    mVolumes.clear();

    status_t res = doUnmount();
    setState(State::kUnmounted);
    return res;
}

/* SPRD: add for UMS @{ */
status_t VolumeBase::share(const std::string& massStorageFilePath) {
    if ((mState != State::kUnmounted) && (mState != State::kUnmountable)) {
        LOG(WARNING) << getId() << " share requires state unmounted or unmountable";
        return -EBUSY;
    }

    if (mType != Type::kPublic) {
        LOG(WARNING) << getId() << " just public volume can be shared";
        return -EBUSY;
    }

    status_t res = doShare(massStorageFilePath);
    if (res == OK) {
        setState(State::kShared);
    }

    return res;
}

status_t VolumeBase::doShare(const std::string& massStorageFilePath) {
    return OK;
}

status_t VolumeBase::doSetState(State state) {
    return OK;
}

status_t VolumeBase::unshare() {
    if (mState != State::kShared) {
        LOG(WARNING) << getId() << " unshare requires state shared";
        return -EBUSY;
    }

    if (mType != Type::kPublic) {
        LOG(WARNING) << getId() << " just public volume can be unshared";
        return -EBUSY;
    }

    status_t res = doUnshare();
    if (res == OK) {
        setState(State::kUnmounted);
    }

    return res;
}

status_t VolumeBase::doUnshare() {
    return OK;
}

std::string VolumeBase::findState(State state) {
    std::string stateStr;
    switch (state) {
    case State::kUnmounted:
        stateStr = "unmounted";
        break;
    case State::kChecking:
        stateStr = "checking";
        break;
    case State::kMounted:
        stateStr = "mounted";
        break;
    case State::kMountedReadOnly:
        stateStr = "mounted_ro";
        break;
    case State::kFormatting:
        stateStr = "formatting";
        break;
    case State::kEjecting:
        stateStr = "ejecting";
        break;
    case State::kUnmountable:
        stateStr = "unmountable";
        break;
    case State::kRemoved:
        stateStr = "removed";
        break;
    case State::kBadRemoval:
        stateStr = "bad_removal";
        break;
    case State::kShared:
        stateStr = "shared";
        break;
    default:
        stateStr = "unknown";
    }
    return stateStr;
}
/* @} */

status_t VolumeBase::format(const std::string& fsType) {
    if (mState == State::kMounted) {
        unmount();
    }

    /* SPRD: add for UMS @{ */
    if (mState == State::kShared) {
        unshare();
    }
    /* @} */

    if ((mState != State::kUnmounted) && (mState != State::kUnmountable)) {
        LOG(WARNING) << getId() << " format requires state unmounted or unmountable";
        return -EBUSY;
    }

    setState(State::kFormatting);
    status_t res = doFormat(fsType);
    setState(State::kUnmounted);
    return res;
}

status_t VolumeBase::doFormat(const std::string& fsType) {
    return -ENOTSUP;
}

}  // namespace vold
}  // namespace android

/*
 *  Copyright (c) 2024, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OFFLOAD_TASKLET_HPP_
#define OFFLOAD_TASKLET_HPP_

#include <stdio.h>

#include <openthread/tasklet.h>

#include "common/non_copyable.hpp"

namespace ot {
namespace Posix {

class Offload;

/**
 * Is used to represent a tasklet.
 *
 */
class Tasklet
{
public:
    /**
     * Implements the tasklet scheduler.
     *
     */
    class Scheduler : private NonCopyable
    {
        friend class Tasklet;

    public:
        /**
         * Initializes the object.
         *
         */
        Scheduler(void)
            : mTail(nullptr)
        {
        }

        /**
         * Indicates whether or not there are tasklets pending.
         *
         * @retval TRUE   If there are tasklets pending.
         * @retval FALSE  If there are no tasklets pending.
         *
         */
        bool AreTaskletsPending(void) const { return mTail != nullptr; }

        /**
         * Processes all tasklets queued when this is called.
         *
         */
        void ProcessQueuedTasklets(void);

    private:
        void PostTasklet(Tasklet &aTasklet);

        Tasklet *mTail; // A circular singly linked-list
    };

    /**
     * Reference is called when the tasklet is run.
     *
     * @param[in]  aTasklet  A reference to the tasklet being run.
     *
     */
    typedef void (&Handler)(Tasklet &aTasklet);

    /**
     * Creates a tasklet instance.
     *
     * @param[in]  aInstance   A reference to the OpenThread instance object.
     * @param[in]  aHandler    A pointer to a function that is called when the tasklet is run.
     *
     */
    Tasklet(Offload &aInstance, Handler aHandler)
        : mInstance(aInstance)
        , mHandler(aHandler)
        , mNext(nullptr)
    {
    }

    /**
     * Puts the tasklet on the tasklet scheduler run queue.
     *
     * If the tasklet is already posted, no change is made and run queue stays as before.
     *
     */
    void Post(void);

    /**
     * Indicates whether the tasklet is posted or not.
     *
     * @retval TRUE  The tasklet is posted.
     * @retval FALSE The tasklet is not posted.
     *
     */
    bool IsPosted(void) const { return (mNext != nullptr); }

    Offload &GetInstance(void) { return mInstance; }

private:
    void RunTask(void) { mHandler(*this); }

    Offload &mInstance;
    Handler  mHandler;
    Tasklet *mNext;
};

/**
 * Defines a tasklet owned by specific type and using a method on owner type as the callback.
 *
 * @tparam Owner              The type of owner of this tasklet.
 * @tparam HandleTaskletPtr   A pointer to a non-static member method of `Owner` to use as tasklet handler.
 *
 * The `Owner` MUST be a type that is accessible using `InstanceLocator::Get<Owner>()`.
 *
 */
template <void (Offload::*HandleTaskletPtr)(void)> class TaskletIn : public Tasklet
{
public:
    /**
     * Initializes the tasklet.
     *
     * @param[in]  aInstance   The OpenThread instance.
     *
     */
    explicit TaskletIn(Offload &aInstance)
        : Tasklet(aInstance, HandleTasklet)
    {
    }

private:
    static void HandleTasklet(Tasklet &aTasklet);
};

template <void (Offload::*HandleTaskletPtr)(void)> void TaskletIn<HandleTaskletPtr>::HandleTasklet(Tasklet &aTasklet)
{
    (aTasklet.GetInstance().*HandleTaskletPtr)();
}

} // namespace Posix
} // namespace ot

#endif // OFFLOAD_TASKLET_HPP_

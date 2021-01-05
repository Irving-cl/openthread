/*
 *  Copyright (c) 2018, The OpenThread Authors.
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

#include "test_platform.h"

#include <openthread/config.h>

#include "test_util.h"
#include "common/code_utils.hpp"
#include "common/instance.hpp"
#include "thread/child_table.hpp"
#include "thread/topology.hpp"

namespace ot {

static ot::Instance *sInstance;

enum
{
    kMaxChildren = OPENTHREAD_CONFIG_MLE_MAX_CHILDREN,
};

struct TestChild
{
    Neighbor::State mState;
    uint16_t        mRloc16;
    otExtAddress    mExtAddress;
};

const Neighbor::StateFilter kAllFilters[] = {
    Neighbor::kInStateValid,
    Neighbor::kInStateValidOrRestoring,
    Neighbor::kInStateChildIdRequest,
    Neighbor::kInStateValidOrAttaching,
    Neighbor::kInStateAnyExceptInvalid,
};

// Checks whether a `Child` matches the `TestChild` struct.
static bool ChildMatches(const Child &aChild, const TestChild &aTestChild)
{
    return (aChild.GetState() == aTestChild.mState) && (aChild.GetRloc16() == aTestChild.mRloc16) &&
           (aChild.GetExtAddress() == static_cast<const Mac::ExtAddress &>(aTestChild.mExtAddress));
}

// Checks whether a `Child::State` matches a `Child::StateFilter`.
static bool StateMatchesFilter(Neighbor::State aState, Neighbor::StateFilter aFilter)
{
    bool  rval = false;
    Child child;

    child.SetState(aState);

    switch (aFilter)
    {
    case Neighbor::kInStateAnyExceptInvalid:
        rval = (aState != Neighbor::kStateInvalid);
        break;

    case Neighbor::kInStateValid:
        rval = (aState == Neighbor::kStateValid);
        break;

    case Neighbor::kInStateValidOrRestoring:
        rval = child.IsStateValidOrRestoring();
        break;

    case Neighbor::kInStateChildIdRequest:
        rval = (aState == Neighbor::kStateChildIdRequest);
        break;

    case Neighbor::kInStateValidOrAttaching:
        rval = child.IsStateValidOrAttaching();
        break;

    case Neighbor::kInStateInvalid:
        rval = child.IsStateInvalid();
        break;

    case Neighbor::kInStateAnyExceptValidOrRestoring:
        rval = !child.IsStateValidOrRestoring();
        break;

    case Neighbor::kInStateAny:
        rval = true;
        break;
    }

    return rval;
}

// Verifies that `ChildTable` contains a given list of `TestChild` entries.
void VerifyChildTableContent(ChildTable &aTable, uint16_t aChildListLength, const TestChild *aChildList)
{
    printf("Test ChildTable with %d entries", aChildListLength);

    for (Neighbor::StateFilter filter : kAllFilters)
    {
        // Verify that we can find all children from given list by rloc or extended address.

        for (uint16_t listIndex = 0; listIndex < aChildListLength; listIndex++)
        {
            Child *      child;
            Mac::Address address;

            if (!StateMatchesFilter(aChildList[listIndex].mState, filter))
            {
                continue;
            }

            child = aTable.FindChild(aChildList[listIndex].mRloc16, filter);
            VerifyOrQuit(child != nullptr, "FindChild(rloc) failed");
            VerifyOrQuit(ChildMatches(*child, aChildList[listIndex]), "FindChild(rloc) returned incorrect child");

            child = aTable.FindChild(static_cast<const Mac::ExtAddress &>(aChildList[listIndex].mExtAddress), filter);
            VerifyOrQuit(child != nullptr, "FindChild(ExtAddress) failed");
            VerifyOrQuit(ChildMatches(*child, aChildList[listIndex]), "FindChild(ExtAddress) returned incorrect child");

            address.SetShort(aChildList[listIndex].mRloc16);
            child = aTable.FindChild(address, filter);
            VerifyOrQuit(child != nullptr, "FindChild(address) failed");
            VerifyOrQuit(ChildMatches(*child, aChildList[listIndex]), "FindChild(address) returned incorrect child");

            address.SetExtended(static_cast<const Mac::ExtAddress &>(aChildList[listIndex].mExtAddress));
            child = aTable.FindChild(address, filter);
            VerifyOrQuit(child != nullptr, "FindChild(address) failed");
            VerifyOrQuit(ChildMatches(*child, aChildList[listIndex]), "FindChild(address) returned incorrect child");
        }

        // Verify `ChildTable::Iterator` behavior.

        {
            ChildTable::Iterator iter(*sInstance, filter);
            bool                 childObserved[kMaxChildren];
            uint16_t             numChildren = 0;

            memset(childObserved, 0, sizeof(childObserved));

            // Use the iterator and verify that each returned `Child` entry is in the expected list.

            for (; !iter.IsDone(); iter++)
            {
                Child *  child    = iter.GetChild();
                Child &  childRef = *iter;
                bool     didFind  = false;
                uint16_t childIndex;

                VerifyOrQuit(child != nullptr, "iter.GetChild() failed");
                VerifyOrQuit(&childRef == child, "iter.operator*() failed");
                VerifyOrQuit(iter->GetRloc16() == child->GetRloc16(), "iter.operator->() failed");

                childIndex = aTable.GetChildIndex(*child);
                VerifyOrQuit(childIndex < aTable.GetMaxChildrenAllowed(), "Child Index is out of bound");
                VerifyOrQuit(aTable.GetChildAtIndex(childIndex) == child, "GetChildAtIndex() failed");

                for (uint16_t index = 0; index < aChildListLength; index++)
                {
                    if (ChildMatches(*iter.GetChild(), aChildList[index]))
                    {
                        childObserved[index] = true;
                        numChildren++;
                        didFind = true;
                        break;
                    }
                }

                VerifyOrQuit(didFind, "ChildTable::Iterator returned an entry not in the expected list");
            }

            // Verify that when iterator is done, it points to `nullptr`.

            VerifyOrQuit(iter.GetChild() == nullptr, "iterator GetChild() failed");

            iter++;
            VerifyOrQuit(iter.IsDone(), "iterator Advance() (after iterator is done) failed");
            VerifyOrQuit(iter.GetChild() == nullptr, "iterator GetChild() failed");

            // Verify that the number of children matches the number of entries we get from iterator.

            VerifyOrQuit(aTable.GetNumChildren(filter) == numChildren, "GetNumChildren() failed");
            VerifyOrQuit(aTable.HasChildren(filter) == (numChildren != 0), "HasChildren() failed");

            // Verify that there is no missing or extra entry between the expected list
            // and what was observed/returned by the iterator.

            for (uint16_t index = 0; index < aChildListLength; index++)
            {
                if (StateMatchesFilter(aChildList[index].mState, filter))
                {
                    VerifyOrQuit(childObserved[index], "iterator failed to return an expected entry");
                }
                else
                {
                    VerifyOrQuit(!childObserved[index], "iterator returned an extra unexpected entry");
                }
            }

            // Verify the behavior of range-based `for` iteration.

            iter.Reset();

            for (Child &child : aTable.Iterate(filter))
            {
                VerifyOrQuit(&child == iter.GetChild(), "range-based for loop Iterate() failed");
                iter++;
            }

            VerifyOrQuit(iter.IsDone(), "range-based for loop Iterate() did not return all entries");
        }
    }

    printf(" -- PASS\n");
}

void TestChildTable(void)
{
    const TestChild testChildList[] = {
        {
            Neighbor::kStateValid,
            0x8001,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x16}},
        },
        {
            Neighbor::kStateParentRequest,
            0x8002,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x17}},
        },
        {
            Neighbor::kStateValid,
            0x8003,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x18}},
        },
        {
            Neighbor::kStateValid,
            0x8004,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x19}},
        },
        {
            Neighbor::kStateRestored,
            0x8005,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x20}},
        },
        {
            Neighbor::kStateValid,
            0x8006,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x21}},
        },
        {
            Neighbor::kStateChildIdRequest,
            0x8007,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x22}},
        },
        {
            Neighbor::kStateChildUpdateRequest,
            0x8008,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x23}},
        },
        {
            Neighbor::kStateParentResponse,
            0x8009,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x24}},
        },
        {
            Neighbor::kStateRestored,
            0x800a,
            {{0x10, 0x20, 0x03, 0x15, 0x10, 0x00, 0x60, 0x25}},
        },
    };

    const uint16_t testListLength = OT_ARRAY_LENGTH(testChildList);

    uint16_t testNumAllowedChildren = 2;

    ChildTable *table;
    otError     error;

    sInstance = testInitInstance();
    VerifyOrQuit(sInstance != nullptr, "Null instance");

    table = &sInstance->Get<ChildTable>();

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    printf("Checking initial state after child table is constructed");

    VerifyOrQuit(table->GetMaxChildrenAllowed() == table->GetMaxChildren(),
                 "GetMaxChildrenAllowed() initial value is incorrect ");

    for (Neighbor::StateFilter filter : kAllFilters)
    {
        VerifyOrQuit(table->HasChildren(filter) == false, "HasChildren() failed after init");
        VerifyOrQuit(table->GetNumChildren(filter) == 0, "GetNumChildren() failed after init");
    }

    printf(" -- PASS\n");

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    VerifyChildTableContent(*table, 0, testChildList);

    VerifyOrQuit(table->GetMaxChildrenAllowed() >= testListLength,
                 "Default child table size is too small for the unit test");

    // Add the child entries from test list one by one and verify the table content
    for (uint16_t i = 0; i < testListLength; i++)
    {
        Child *child;

        child = table->GetNewChild();
        VerifyOrQuit(child != nullptr, "GetNewChild() failed");

        child->SetState(testChildList[i].mState);
        child->SetRloc16(testChildList[i].mRloc16);
        child->SetExtAddress((static_cast<const Mac::ExtAddress &>(testChildList[i].mExtAddress)));

        VerifyChildTableContent(*table, i + 1, testChildList);
    }

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Verify Child Table clear

    table->Clear();

    VerifyChildTableContent(*table, 0, testChildList);

    // Add the child entries from test list in reverse order and verify the table content
    for (uint16_t i = testListLength; i > 0; i--)
    {
        Child *child;

        child = table->GetNewChild();
        VerifyOrQuit(child != nullptr, "GetNewChild() failed");

        child->SetState(testChildList[i - 1].mState);
        child->SetRloc16(testChildList[i - 1].mRloc16);
        child->SetExtAddress((static_cast<const Mac::ExtAddress &>(testChildList[i - 1].mExtAddress)));

        VerifyChildTableContent(*table, testListLength - i + 1, &testChildList[i - 1]);
    }

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    printf("Test Get/SetMaxChildrenAllowed");

    error = table->SetMaxChildrenAllowed(kMaxChildren - 1);
    VerifyOrQuit(error == OT_ERROR_INVALID_STATE, "SetMaxChildrenAllowed() should fail when table is not empty");

    table->Clear();
    error = table->SetMaxChildrenAllowed(kMaxChildren + 1);
    VerifyOrQuit(error == OT_ERROR_INVALID_ARGS, "SetMaxChildrenAllowed() did not fail with an invalid arg");

    error = table->SetMaxChildrenAllowed(0);
    VerifyOrQuit(error == OT_ERROR_INVALID_ARGS, "SetMaxChildrenAllowed() did not fail with an invalid arg");

    error = table->SetMaxChildrenAllowed(testNumAllowedChildren);
    VerifyOrQuit(error == OT_ERROR_NONE, "SetMaxChildrenAllowed() failed");
    VerifyOrQuit(table->GetMaxChildrenAllowed() == testNumAllowedChildren, "GetMaxChildrenAllowed() failed");

    for (uint16_t num = 0; num < testNumAllowedChildren; num++)
    {
        Child *child = table->GetNewChild();

        VerifyOrQuit(child != nullptr, "GetNewChild() failed");
        child->SetState(Neighbor::kStateValid);
    }

    VerifyOrQuit(table->GetNewChild() == nullptr, "GetNewChild() did not fail when table was full");

    printf(" -- PASS\n");

    testFreeInstance(sInstance);
}

} // namespace ot

int main(void)
{
    ot::TestChildTable();
    printf("\nAll tests passed.\n");
    return 0;
}

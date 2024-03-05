
#include "offload.hpp"

#include "posix-offload.h"

#include "common/code_utils.hpp"
#include "common/new.hpp"
#include "lib/spinel/ncp_spinel.hpp"
#include "posix/platform/spinel.hpp"

namespace ot {
namespace Posix {

// Define the raw storage used for OpenThread offload instance.
OT_DEFINE_ALIGNED_VAR(gInstanceRaw, sizeof(Offload), uint64_t);

Offload &Offload::InitSingle(void)
{
    Offload *instance = &Get();

    VerifyOrExit(!instance->mIsInitialized);

    instance = new (&gInstanceRaw) Offload();

    instance->AfterInit();

exit:
    return *instance;
}

Offload &Offload::Get(void)
{
    void *instance = &gInstanceRaw;

    return *static_cast<Offload *>(instance);
}

otError Offload::ActiveScan(uint32_t                 aScanChannels,
                            uint16_t                 aScanDuration,
                            otHandleActiveScanResult aHandler,
                            void                    *aContext)
{
    (void)aScanChannels;
    (void)aScanDuration;

    otError error = kErrorNone;

    VerifyOrExit(!mIsActiveScanInProgress, error = kErrorBusy);
    VerifyOrExit(aHandler != nullptr && aContext != nullptr, error = kErrorInvalidArgs);

    mActiveScanHandler.Set(aHandler, aContext);
    SuccessOrExit(error = mNcpSpinel.ActiveScan());

    mIsActiveScanInProgress = true;

exit:
    return error;
}

otDeviceRole Offload::GetDeviceRole(void)
{
    otDeviceRole role;

    mNcpSpinel.GetDeviceRole(role);

    return role;
}

otError Offload::DatasetCreateNewNetwork(otOperationalDataset &aDataset)
{
    return mNcpSpinel.DatasetInitNew(&aDataset);
}

otError Offload::Ip6SetEnabled(bool aEnabled) { return mNcpSpinel.Ip6SetEnabled(aEnabled); }

otError Offload::ThreadSetEnabled(bool aEnabled) { return mNcpSpinel.ThreadSetEnabled(aEnabled); }

otError Offload::Ip6Send(const uint8_t *aPacket, size_t aLen) { return mNcpSpinel.Ip6Send(aPacket, aLen); }

void Offload::Ip6SetAddressCallback(otIp6AddressCallback aCallback, void *aContext)
{
    mAddressCallback.Set(aCallback, aContext);
}

void Offload::Ip6SetReceiveCallback(Ip6ReceiveCallback aCallback, void *aContext)
{
    mIp6ReceiveCallback.Set(aCallback, aContext);
}

void Offload::NetifSetStateChangedCallback(otStateChangedCallback aCallback, void *aContext)
{
    mNetifStateChangedCallback.Set(aCallback, aContext);
}

Offload::Offload(void)
    : mNcpSpinel(this, GetSpinelBase())
    , mIsInitialized(false)
    , mUpdateUnicastAddressTableTask(*this)
{
}

void Offload::AfterInit(void)
{
    struct ot::Spinel::NcpSpinelCallbacks callbacks = {&Offload::ActiveScanDone, &Offload::UpdateIp6AddressTable,
                                                       &Offload::ReceiveIp6};

    mNcpSpinel.Init(callbacks);
    mIsInitialized = true;
}

void Offload::TaskletsProcess(void) { mTaskletScheduler.ProcessQueuedTasklets(); }

void Offload::ActiveScanDone(otInstance *aInstance, otActiveScanResult *aScanResult)
{
    static_cast<Offload *>(aInstance)->ActiveScanDone(aScanResult);
}

void Offload::ActiveScanDone(otActiveScanResult *aScanResult) { mActiveScanHandler.InvokeIfSet(aScanResult); }

void Offload::UpdateIp6AddressTable(otInstance *aInstance, otNetifAddress *aAddressTable, uint8_t aSize)
{
    static_cast<Offload *>(aInstance)->UpdateIp6AddressTable(aAddressTable, aSize);
}

void Offload::UpdateIp6AddressTable(otNetifAddress *aAddressTable, uint8_t aSize)
{
    OT_ASSERT(aAddressTable != nullptr);

    // Clear Pending Addresses
    while (!mUnicastAddressTablePending.IsEmpty())
    {
        Ip6::Netif::UnicastAddress *address = mUnicastAddressTablePending.Pop();
        mUnicastAddressPool.Free(*address);
    }

    for (uint8_t i = 0; i < aSize; i++)
    {
        Ip6::Netif::UnicastAddress *address = mUnicastAddressPool.Allocate();
        address->mAddress                   = aAddressTable[i].mAddress;
        address->mPrefixLength              = aAddressTable[i].mPrefixLength;
        address->mScopeOverride             = aAddressTable[i].mScopeOverride;
        address->mScopeOverrideValid        = true;
        address->mPreferred                 = aAddressTable[i].mPreferred;
        address->mValid                     = aAddressTable[i].mValid;

        mUnicastAddressTablePending.Add(*address);
    }

    mUpdateUnicastAddressTableTask.Post();
}

void Offload::ReceiveIp6(otInstance *aInstance, const uint8_t *aData, uint16_t aLength)
{
    static_cast<Offload *>(aInstance)->ReceiveIp6(aData, aLength);
}

void Offload::ReceiveIp6(const uint8_t *aData, uint16_t aLength) { mIp6ReceiveCallback.InvokeIfSet(aData, aLength); }

void Offload::UpdateUnicastAddressTable(void)
{
    for (const Ip6::Netif::UnicastAddress &address : mUnicastAddressTable)
    {
        if (!mUnicastAddressTablePending.ContainsMatching(address.GetAddress()))
        {
            otIp6AddressInfo info;

            info.mAddress      = &address.mAddress;
            info.mPrefixLength = address.mPrefixLength;
            info.mScope        = address.GetScope();
            info.mPreferred    = address.mPreferred;

            otLogInfoPlat("Remove address %s, prefixLen:%u, scope:%u, preferred:%u",
                          address.GetAddress().ToString().AsCString(), info.mPrefixLength, info.mScope,
                          info.mPreferred);
            mAddressCallback.InvokeIfSet(&info, false);
        }
    }

    for (const Ip6::Netif::UnicastAddress &address : mUnicastAddressTablePending)
    {
        if (!mUnicastAddressTable.ContainsMatching(address.GetAddress()))
        {
            otIp6AddressInfo info;

            info.mAddress      = &address.mAddress;
            info.mPrefixLength = address.mPrefixLength;
            info.mScope        = address.GetScope();
            info.mPreferred    = address.mPreferred;

            otLogInfoPlat("Add address %s, prefixLen:%u, scope:%u, preferred:%u",
                          address.GetAddress().ToString().AsCString(), info.mPrefixLength, info.mScope,
                          info.mPreferred);
            mAddressCallback.InvokeIfSet(&info, true);
        }
    }

    LinkedList<Ip6::Netif::UnicastAddress> tmp = mUnicastAddressTable;
    mUnicastAddressTable                       = mUnicastAddressTablePending;
    mUnicastAddressTablePending                = tmp;

    if ((!mUnicastAddressTable.IsEmpty() && mUnicastAddressTablePending.IsEmpty()) ||
        (mUnicastAddressTable.IsEmpty() && !mUnicastAddressTablePending.IsEmpty()))
    {
        mNetifStateChangedCallback.InvokeIfSet(OT_CHANGED_THREAD_NETIF_STATE);
    }
}

Offload *offloadInstanceInit(void) { return &Offload::InitSingle(); }

} // namespace Posix
} // namespace ot

void offloadTaskletsProcess(otInstance *aInstance) { static_cast<ot::Posix::Offload *>(aInstance)->TaskletsProcess(); }

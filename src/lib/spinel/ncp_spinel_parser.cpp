
#include "ncp_spinel_parser.hpp"

#include <string.h>

#include "common/code_utils.hpp"
#include "common/log.hpp"
#include "common/string.hpp"
#include "lib/spinel/spinel_decoder.hpp"

namespace ot {
namespace Spinel {

static constexpr uint8_t kDefaultFlags                   = 0xff;
static constexpr uint8_t kObtainNetworkKeyMask           = 1 << 7;
static constexpr uint8_t kNativeCommissioningMask        = 1 << 6;
static constexpr uint8_t kRoutersMask                    = 1 << 5;
static constexpr uint8_t kExternalCommissioningMask      = 1 << 4;
static constexpr uint8_t kBeaconsMask                    = 1 << 3;
static constexpr uint8_t kCommercialCommissioningMask    = 1 << 2;
static constexpr uint8_t kAutonomousEnrollmentMask       = 1 << 1;
static constexpr uint8_t kNetworkKeyProvisioningMask     = 1 << 0;
static constexpr uint8_t kTobleLinkMask                  = 1 << 7;
static constexpr uint8_t kNonCcmRoutersMask              = 1 << 6;
static constexpr uint8_t kReservedMask                   = 0x38;
static constexpr uint8_t kVersionThresholdForRoutingMask = 0x07;

void SetSecurityPolicyFromFlags(const uint8_t *aFlags, uint8_t aFlagsLength, otSecurityPolicy *aSecurityPolicy)
{

    aSecurityPolicy->mObtainNetworkKeyEnabled        = aFlags[0] & kObtainNetworkKeyMask;
    aSecurityPolicy->mNativeCommissioningEnabled     = aFlags[0] & kNativeCommissioningMask;
    aSecurityPolicy->mRoutersEnabled                 = aFlags[0] & kRoutersMask;
    aSecurityPolicy->mExternalCommissioningEnabled   = aFlags[0] & kExternalCommissioningMask;
    //aSecurityPolicy->mBeaconsEnabled                 = aFlags[0] & kBeaconsMask;
    aSecurityPolicy->mCommercialCommissioningEnabled = (aFlags[0] & kCommercialCommissioningMask) == 0;
    aSecurityPolicy->mAutonomousEnrollmentEnabled    = (aFlags[0] & kAutonomousEnrollmentMask) == 0;
    aSecurityPolicy->mNetworkKeyProvisioningEnabled  = (aFlags[0] & kNetworkKeyProvisioningMask) == 0;

    VerifyOrExit(aFlagsLength > sizeof(aFlags[0]));
    aSecurityPolicy->mTobleLinkEnabled           = aFlags[1] & kTobleLinkMask;
    aSecurityPolicy->mNonCcmRoutersEnabled       = (aFlags[1] & kNonCcmRoutersMask) == 0;
    aSecurityPolicy->mVersionThresholdForRouting = aFlags[1] & kVersionThresholdForRoutingMask;

exit:
    return;
}

otError ParseOperationalDataset(uint8_t *aBuf, uint8_t aLen, otOperationalDataset *aOpDataset)
{
    otError error = OT_ERROR_NONE;
    Decoder decoder;

    VerifyOrExit(aBuf != nullptr, error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(aOpDataset != nullptr, error = OT_ERROR_INVALID_ARGS);

    memset(aOpDataset, 0, sizeof(otOperationalDataset));
    decoder.Init(aBuf, aLen);

    while (!decoder.IsAllReadInStruct())
    {
        unsigned int propKey;

        SuccessOrExit((error = decoder.OpenStruct()));
        SuccessOrExit((error = decoder.ReadUintPacked(propKey)));

        switch (static_cast<spinel_prop_key_t>(propKey))
        {
        case SPINEL_PROP_DATASET_ACTIVE_TIMESTAMP:
        {
            SuccessOrExit((error = decoder.ReadUint64(aOpDataset->mActiveTimestamp.mSeconds)));
            aOpDataset->mComponents.mIsActiveTimestampPresent = true;
            break;
        }
        case SPINEL_PROP_DATASET_PENDING_TIMESTAMP:
        {
            SuccessOrExit((error = decoder.ReadUint64(aOpDataset->mPendingTimestamp.mSeconds)));
            aOpDataset->mComponents.mIsPendingTimestampPresent = true;
            break;
        }
        case SPINEL_PROP_NET_NETWORK_KEY:
        {
            const uint8_t *key;
            uint16_t       len;

            SuccessOrExit((error = decoder.ReadData(key, len)));
            VerifyOrExit(len == OT_NETWORK_KEY_SIZE, error = OT_ERROR_INVALID_STATE);
            memcpy(aOpDataset->mNetworkKey.m8, key, len);
            aOpDataset->mComponents.mIsNetworkKeyPresent = true;
            break;
        }

        case SPINEL_PROP_NET_NETWORK_NAME:
        {
            const char *name;
            size_t      len;

            SuccessOrExit((error = decoder.ReadUtf8(name)));
            len = StringLength(name, OT_NETWORK_NAME_MAX_SIZE);
            memcpy(aOpDataset->mNetworkName.m8, name, len);
            aOpDataset->mNetworkName.m8[len]              = '\0';
            aOpDataset->mComponents.mIsNetworkNamePresent = true;
            break;
        }

        case SPINEL_PROP_NET_XPANID:
        {
            const uint8_t *xpanid;
            uint16_t       len;

            SuccessOrExit(error = decoder.ReadData(xpanid, len));
            VerifyOrExit(len == OT_EXT_PAN_ID_SIZE, error = OT_ERROR_INVALID_STATE);
            memcpy(aOpDataset->mExtendedPanId.m8, xpanid, len);
            aOpDataset->mComponents.mIsExtendedPanIdPresent = true;
            break;
        }

        case SPINEL_PROP_IPV6_ML_PREFIX:
        {
            const otIp6Address *addr;
            uint8_t             prefixLen;

            SuccessOrExit((error = decoder.ReadIp6Address(addr)));
            SuccessOrExit((error = decoder.ReadUint8(prefixLen)));
            VerifyOrExit(prefixLen == OT_IP6_PREFIX_BITSIZE, error = OT_ERROR_INVALID_STATE);
            memcpy(aOpDataset->mMeshLocalPrefix.m8, addr, OT_MESH_LOCAL_PREFIX_SIZE);
            aOpDataset->mComponents.mIsMeshLocalPrefixPresent = true;
            break;
        }

        case SPINEL_PROP_DATASET_DELAY_TIMER:
        {
            SuccessOrExit((error = decoder.ReadUint32(aOpDataset->mDelay)));
            aOpDataset->mComponents.mIsDelayPresent = true;
            break;
        }

        case SPINEL_PROP_MAC_15_4_PANID:
        {
            SuccessOrExit((error = decoder.ReadUint16(aOpDataset->mPanId)));
            aOpDataset->mComponents.mIsPanIdPresent = true;
            break;
        }

        case SPINEL_PROP_PHY_CHAN:
        {
            uint8_t channel;

            SuccessOrExit((error = decoder.ReadUint8(channel)));
            aOpDataset->mChannel                      = channel;
            aOpDataset->mComponents.mIsChannelPresent = true;
            break;
        }

        case SPINEL_PROP_NET_PSKC:
        {
            const uint8_t *psk;
            uint16_t       len;

            SuccessOrExit(error = decoder.ReadData(psk, len));
            VerifyOrExit(len == OT_PSKC_MAX_SIZE, error = OT_ERROR_INVALID_STATE);
            memcpy(aOpDataset->mPskc.m8, psk, OT_PSKC_MAX_SIZE);
            aOpDataset->mComponents.mIsPskcPresent = true;
            break;
        }

        case SPINEL_PROP_DATASET_SECURITY_POLICY:
        {
            uint8_t flags[2];
            uint8_t flagsLength = 2;

            SuccessOrExit((error = decoder.ReadUint16(aOpDataset->mSecurityPolicy.mRotationTime)));
            SuccessOrExit((error = decoder.ReadUint8(flags[0])));
            SuccessOrExit((error = decoder.ReadUint8(flags[1])));

            SetSecurityPolicyFromFlags(flags, flagsLength, &aOpDataset->mSecurityPolicy);
            aOpDataset->mComponents.mIsSecurityPolicyPresent = true;
            break;
        }

        case SPINEL_PROP_PHY_CHAN_SUPPORTED:
        {
            uint8_t channel;

            aOpDataset->mChannelMask = 0;

            while (!decoder.IsAllReadInStruct())
            {
                SuccessOrExit((error = decoder.ReadUint8(channel)));
                VerifyOrExit(channel <= 31, error = OT_ERROR_INVALID_STATE);
                aOpDataset->mChannelMask |= (1UL << channel);
            }
            aOpDataset->mComponents.mIsChannelMaskPresent = true;
            break;
        }

        default:
            break;
        }

        SuccessOrExit((error = decoder.CloseStruct()));
    }

exit:
    return error;
}

otError ParseIp6Addresses(const uint8_t *aBuf, uint8_t aLen, otNetifAddress *aAddressList, uint8_t &aListLen)
{
    otError error = OT_ERROR_NONE;
    Decoder decoder;
    uint8_t index = 0;

    VerifyOrExit(aBuf != nullptr, error = OT_ERROR_INVALID_ARGS);

    decoder.Init(aBuf, aLen);

    while (!decoder.IsAllReadInStruct())
    {
        VerifyOrExit(index < aListLen, error = OT_ERROR_NO_BUFS);

        otNetifAddress *cur = &aAddressList[index];
        const otIp6Address *addr;
        uint8_t prefixLength;
        uint32_t preferred;
        uint32_t valid;

        SuccessOrExit(error = decoder.OpenStruct());
        SuccessOrExit(error = decoder.ReadIp6Address(addr));
        cur->mAddress = *addr;
        SuccessOrExit(error = decoder.ReadUint8(prefixLength));
        cur->mPrefixLength = prefixLength;
        SuccessOrExit(error = decoder.ReadUint32(preferred));
        cur->mPreferred = preferred ? true : false;
        SuccessOrExit(error = decoder.ReadUint32(valid));
        cur->mValid = valid ? true : false;
        // TODO: workaround
        cur->mScopeOverrideValid = false;

        SuccessOrExit((error = decoder.CloseStruct()));
        index++;
    }

exit:
    aListLen = index;
    return error;
}

} // Spinel
} // ot

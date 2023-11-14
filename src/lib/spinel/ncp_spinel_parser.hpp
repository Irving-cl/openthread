
#include <openthread/dataset.h>
#include <openthread/error.h>

namespace ot {
namespace Spinel {

otError ParseOperationalDataset(uint8_t *aBuf, uint8_t aLen, otOperationalDataset *aDataset);

otError ParseIp6Addresses(const uint8_t *aBuf, uint8_t aLen, otNetifAddress *aAddressList, uint8_t &aListLen);

} // Spinel
} // ot

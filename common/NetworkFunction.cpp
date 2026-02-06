/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/NetworkFunction.h"
#include <cassert>

using namespace NetworkAnalytical;

Bandwidth NetworkAnalytical::bw_GBps_to_Bpns(const Bandwidth bw_GBps) noexcept {
    assert(bw_GBps > 0);

    // 1 GB/s = 1e9 B/s = 1 B/ns (using decimal GB, not binary GiB)
    // Previously used (1 << 30) which is GiB, causing 7.37% bandwidth inflation
    return bw_GBps;
}

#ifndef CAN_ID_MANAGER_H
#define CAN_ID_MANAGER_H

#include <array>
#include "isotp.h"

template <typename... UInt8s>
class CanLinkManager {
private:
    static constexpr std::size_t N = sizeof...(UInt8s);
    /* bit 10: 1 for ISOTP CAN frame, 0 for non-ISOTP CAN frame;
     * bits 9-5: sender addr;
     * bits 4-0: receiver addr
     */
    static constexpr uint8_t k_numCanAddrBits_ = 5; 
    static constexpr uint8_t k_canAddrMask_ = (1 << k_numCanAddrBits_) - 1; //0x1F
    static_assert(N <= (1 << k_numCanAddrBits_));

    uint8_t myCanAddr_;
    std::array<IsoTpLink, N> isotpLinks_;

public:
    CanLinkManager(uint8_t myCanAddr, UInt8s... peerCanAddrs): myCanAddr_(myCanAddr) {
        std::array<uint8_t, N> peerAddrs{peerCanAddrs...};
        for (std::size_t idx = 0; idx < N; ++idx) {
            isotp_init_link(&isotpLinks_[idx], MakeSendCanId(peerAddrs[idx]), MakeReceiveCanId(peerAddrs[idx]));
        }
    }

    std::array<IsoTpLink, N>& GetIsotpLinks() {return isotpLinks_;}

    IsoTpLink* GetLinkFromReceiveCanId(uint16_t receiveCanId) {
        for (std::size_t idx = 0; idx < N; ++idx) {
            if (isotpLinks_[idx].receive_arbitration_id == receiveCanId) {
                return &isotpLinks_[idx];
            }
        }

        return nullptr;
    }

private:
    /* bit 10: 1 for ISOTP CAN frame, 0 for non-ISOTP CAN frame;
     * bits 9-5: sender addr;
     * bits 4-0: receiver addr
     */

    uint16_t MakeReceiveCanId(uint8_t peerCanAddr) {
        uint16_t canId = 1 << (k_numCanAddrBits_ * 2);
        /* I am the receiver */
        return canId | ((peerCanAddr & k_canAddrMask_) << k_numCanAddrBits_)
               | (myCanAddr_ & k_canAddrMask_);
    }

    uint16_t MakeSendCanId(uint8_t peerCanAddr) {
        uint16_t canId = 1 << (k_numCanAddrBits_ * 2);
        /* I am the sender */
        return canId | ((myCanAddr_ & k_canAddrMask_) << k_numCanAddrBits_)
               | (peerCanAddr & k_canAddrMask_);
    }
};

/* Deduction guide (C++17+) so that we can write, e.g.:
 * CanLinkManager canManagers(0x01, 0x10, 0x11);
 * The above defines my CAN addr as 0x01, it communicates
 * with two peer nodes that have CAN addrs 0x10 and 0x11
 */
template <typename... UInt8s>
CanLinkManager(uint8_t, UInt8s...) -> CanLinkManager<UInt8s...>;

#endif //CAN_ID_MANAGER_H

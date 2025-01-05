#include <stdint.h>
#include "assert.h"
#include "isotp.h"

///////////////////////////////////////////////////////
///                 STATIC FUNCTIONS                ///
///////////////////////////////////////////////////////

/* st_min to microsecond */
static uint8_t isotp_us_to_st_min(uint32_t us) {
    if (us <= 127000) {
        if (us >= 100 && us <= 900) {
            return (uint8_t)(0xF0 + (us / 100));
        } else {
            return (uint8_t)(us / 1000u);
        }
    }

    return 0;
}

/* st_min to usec  */
static uint32_t isotp_st_min_to_us(uint8_t st_min) {
    if (st_min <= 0x7F) {
        return st_min * 1000;
    } else if (st_min >= 0xF1 && st_min <= 0xF9) {
        return (st_min - 0xF0) * 100;
    }
    return 0;
}

static int isotp_send_flow_control(const IsoTpLink* link, uint8_t flow_status, uint8_t block_size, uint32_t st_min_us) {

    IsoTpCanMessage message;
    int ret;
    uint8_t size = 0;

    /* setup message  */
    message.as.flow_control.type = ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME;
    message.as.flow_control.FS = flow_status;
    message.as.flow_control.BS = block_size;
    message.as.flow_control.STmin = isotp_us_to_st_min(st_min_us);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.flow_control.reserve, ISO_TP_FRAME_PADDING_VALUE, sizeof(message.as.flow_control.reserve));
    size = sizeof(message);
#else
    size = 3;
#endif

    ret = isotp_user_send_can(link->send_arbitration_id, message.as.data_array.ptr, size
    #if defined (ISO_TP_USER_SEND_CAN_ARG)
    ,link->user_send_can_arg
    #endif
    );

    return ret;
}

static int isotp_send_single_frame(const IsoTpLink* link) {

    IsoTpCanMessage message;
    int ret;
    uint8_t size = 0;

    /* multi frame message length must greater than 7  */
    assert(link->send_size <= 7);

    /* setup message  */
    message.as.single_frame.type = ISOTP_PCI_TYPE_SINGLE;
    message.as.single_frame.SF_DL = (uint8_t) link->send_size;
    (void) memcpy(message.as.single_frame.data, link->send_buffer, link->send_size);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.single_frame.data + link->send_size, ISO_TP_FRAME_PADDING_VALUE, sizeof(message.as.single_frame.data) - link->send_size);
    size = sizeof(message);
#else
    size = link->send_size + 1;
#endif

    ret = isotp_user_send_can(link->send_arbitration_id, message.as.data_array.ptr, size
    #if defined (ISO_TP_USER_SEND_CAN_ARG)
    ,link->user_send_can_arg
    #endif
    );

    return ret;
}

static int isotp_send_first_frame(IsoTpLink* link) {
    
    IsoTpCanMessage message;
    int ret;

    /* multi frame message length must greater than 7  */
    assert(link->send_size > 7);

    /* setup message  */
    message.as.first_frame.type = ISOTP_PCI_TYPE_FIRST_FRAME;
    message.as.first_frame.FF_DL_low = (uint8_t) link->send_size;
    message.as.first_frame.FF_DL_high = (uint8_t) (0x0F & (link->send_size >> 8));
    (void) memcpy(message.as.first_frame.data, link->send_buffer, sizeof(message.as.first_frame.data));

    /* send message */
    ret = isotp_user_send_can(link->send_arbitration_id, message.as.data_array.ptr, sizeof(message) 
    #if defined (ISO_TP_USER_SEND_CAN_ARG)
    ,link->user_send_can_arg
    #endif

    );
    if (ISOTP_RET_OK == ret) {
        link->send_offset += sizeof(message.as.first_frame.data);
        link->send_sn = 1;
    }

    return ret;
}

static int isotp_send_consecutive_frame(IsoTpLink* link) {
    
    IsoTpCanMessage message;
    uint16_t data_length;
    int ret;
    uint8_t size = 0;

    /* multi frame message length must greater than 7  */
    assert(link->send_size > 7);

    /* setup message  */
    message.as.consecutive_frame.type = TSOTP_PCI_TYPE_CONSECUTIVE_FRAME;
    message.as.consecutive_frame.SN = link->send_sn;
    data_length = link->send_size - link->send_offset;
    if (data_length > sizeof(message.as.consecutive_frame.data)) {
        data_length = sizeof(message.as.consecutive_frame.data);
    }
    (void) memcpy(message.as.consecutive_frame.data, link->send_buffer + link->send_offset, data_length);

    /* send message */
#ifdef ISO_TP_FRAME_PADDING
    (void) memset(message.as.consecutive_frame.data + data_length, ISO_TP_FRAME_PADDING_VALUE, sizeof(message.as.consecutive_frame.data) - data_length);
    size = sizeof(message);
#else
    size = data_length + 1;
#endif

    ret = isotp_user_send_can(link->send_arbitration_id,
            message.as.data_array.ptr, size
#if defined (ISO_TP_USER_SEND_CAN_ARG)
    ,link->user_send_can_arg
#endif
    );

    if (ISOTP_RET_OK == ret) {
        link->send_offset += data_length;
        if (++(link->send_sn) > 0x0F) {
            link->send_sn = 0;
        }
    }
    
    return ret;
}

static int isotp_receive_single_frame(IsoTpLink* link, const IsoTpCanMessage* message, uint8_t len) {
    /* check data length */
    if ((0 == message->as.single_frame.SF_DL) || (message->as.single_frame.SF_DL > (len - 1))) {
        isotp_user_debug("Single-frame length too small.");
        return ISOTP_RET_LENGTH;
    }

    /* copying data */
    (void) memcpy(link->receive_buffer, message->as.single_frame.data, message->as.single_frame.SF_DL);
    link->receive_size = message->as.single_frame.SF_DL;
    
    return ISOTP_RET_OK;
}

static int isotp_receive_first_frame(IsoTpLink *link, IsoTpCanMessage *message, uint8_t len) {
    uint16_t payload_length;

    if (8 != len) {
        isotp_user_debug("First frame should be 8 bytes in length.");
        return ISOTP_RET_LENGTH;
    }

    /* check data length */
    payload_length = message->as.first_frame.FF_DL_high;
    payload_length = (payload_length << 8) + message->as.first_frame.FF_DL_low;

    /* should not use multiple frame transmition */
    if (payload_length <= 7) {
        isotp_user_debug("Should not use multiple frame transmission.");
        return ISOTP_RET_LENGTH;
    }
    
    if (payload_length > link->receive_buf_size) {
        isotp_user_debug("Multi-frame response too large for receiving buffer.");
        return ISOTP_RET_OVERFLOW;
    }
    
    /* copying data */
    (void) memcpy(link->receive_buffer, message->as.first_frame.data, sizeof(message->as.first_frame.data));
    link->receive_size = payload_length;
    link->receive_offset = sizeof(message->as.first_frame.data);
    link->receive_sn = 1;

    return ISOTP_RET_OK;
}

static int isotp_receive_consecutive_frame(IsoTpLink *link, IsoTpCanMessage *message, uint8_t len) {
    uint16_t remaining_bytes;
    
    /* check sn */
    if (link->receive_sn != message->as.consecutive_frame.SN) {
        return ISOTP_RET_WRONG_SN;
    }

    /* check data length */
    remaining_bytes = link->receive_size - link->receive_offset;
    if (remaining_bytes > sizeof(message->as.consecutive_frame.data)) {
        remaining_bytes = sizeof(message->as.consecutive_frame.data);
    }
    if (remaining_bytes > len - 1) {
        isotp_user_debug("Consecutive frame too short.");
        return ISOTP_RET_LENGTH;
    }

    /* copying data */
    (void) memcpy(link->receive_buffer + link->receive_offset, message->as.consecutive_frame.data, remaining_bytes);

    link->receive_offset += remaining_bytes;
    if (++(link->receive_sn) > 0x0F) {
        link->receive_sn = 0;
    }

    return ISOTP_RET_OK;
}

static int isotp_receive_flow_control_frame(IsoTpLink *link, IsoTpCanMessage *message, uint8_t len) {
    /* unused args */
    (void) link;
    (void) message;

    /* check message length */
    if (len < 3) {
        isotp_user_debug("Flow control frame too short.");
        return ISOTP_RET_LENGTH;
    }

    return ISOTP_RET_OK;
}

///////////////////////////////////////////////////////
///                 PUBLIC FUNCTIONS                ///
///////////////////////////////////////////////////////

int isotp_send(IsoTpLink *link, const uint8_t payload[], uint16_t size) {
    int ret;

    if (link == 0x0) {
        isotp_user_debug("Link is null!");
        return 0;
    }

    if (size > link->send_buf_size) {
        isotp_user_debug("Message size too large. Increase ISO_TP_MAX_MESSAGE_SIZE to set a larger buffer\n");
        char message[128];
        sprintf(&message[0], "Attempted to send %d bytes; max size is %d!\n", size, link->send_buf_size);
        isotp_user_debug(message);
        return 0;
    }

    if (ISOTP_SEND_STATUS_IDLE != link->send_status) {
        isotp_user_debug("Can only send when send status is in IDLE!");
        return 0;
    }

    /* copy into local buffer */
    link->send_size = size;
    link->send_offset = 0;
    (void) memcpy(link->send_buffer, payload, size);

    if (link->send_size < 8) {
        /* send single frame */
        ret = isotp_send_single_frame(link);
    } else {
        /* send multi-frame */
        ret = isotp_send_first_frame(link);

        /* init multi-frame control flags */
        if (ISOTP_RET_OK == ret) {
            link->send_bs_remain = 0;
            link->send_st_min_us = 0;
            link->send_wtf_count = 0;
            link->send_timer_st = isotp_user_get_us();
            link->send_timer_bs = isotp_user_get_us() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US;
            link->send_protocol_result = ISOTP_PROTOCOL_RESULT_OK;
            link->send_status = ISOTP_SEND_STATUS_INPROGRESS;
        }
    }

    return ret == ISOTP_RET_OK ? 1 : 0;
}

int isotp_on_can_message(IsoTpLink *link, const uint8_t *data, uint8_t len) {
    IsoTpCanMessage message;
    int ret;
    int needStartTimer = 0;
    
    if (len < 2 || len > 8) {
        return needStartTimer;
    }

    memcpy(message.as.data_array.ptr, data, len);
    memset(message.as.data_array.ptr + len, 0, sizeof(message.as.data_array.ptr) - len);

    switch (message.as.common.type) {
        case ISOTP_PCI_TYPE_SINGLE: {
            /* Can only receive when the receive_status is IDLE. If the receiving status
             * is INPROGRESS while no further incoming packets, the isotp_poll function
             * will set the receive_status to IDLE when timeout. If in FULL status, should
             * call isotp_receive to retrieve the previous packet before overide the 
             * receieve buffer.
             */
            if (ISOTP_RECEIVE_STATUS_IDLE != link->receive_status) {
                link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
                break;
            }

            /* update protocol result */
            link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_OK;

            /* handle message */
            ret = isotp_receive_single_frame(link, &message, len);
            
            if (ISOTP_RET_OK == ret) {
                /* change status */
                link->receive_status = ISOTP_RECEIVE_STATUS_FULL;
            }
            break;
        }
        case ISOTP_PCI_TYPE_FIRST_FRAME: {
            /* Can only receive when the receive_status is IDLE. If the receiving status
             * is INPROGRESS while no further incoming packets, the isotp_poll function
             * will set the receive_status to IDLE when timeout. If in FULL status, should
             * call isotp_receive to retrieve the previous packet before overide the 
             * receieve buffer.
             */
            if (ISOTP_RECEIVE_STATUS_IDLE != link->receive_status) {
                link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
                break;
            }

            /* update protocol result */
            link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_OK;

            /* handle message */
            ret = isotp_receive_first_frame(link, &message, len);

            /* if overflow happened */
            if (ISOTP_RET_OVERFLOW == ret) {
                /* update protocol result */
                link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW;
                /* change status */
                link->receive_status = ISOTP_RECEIVE_STATUS_IDLE;
                /* send error message */
                isotp_send_flow_control(link, PCI_FLOW_STATUS_OVERFLOW, 0, 0);
                break;
            }

            /* if receive successful */
            if (ISOTP_RET_OK == ret) {
                /* change status */
                link->receive_status = ISOTP_RECEIVE_STATUS_INPROGRESS;
                /* send fc frame */
                link->receive_bs_count = ISO_TP_DEFAULT_BLOCK_SIZE;
                isotp_send_flow_control(link, PCI_FLOW_STATUS_CONTINUE, link->receive_bs_count, ISO_TP_DEFAULT_ST_MIN_US);
                /* refresh timer cs */
                link->receive_timer_cr = isotp_user_get_us() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US;

                needStartTimer = 1;
            }
            
            break;
        }
        case TSOTP_PCI_TYPE_CONSECUTIVE_FRAME: {
            /* check if in receiving status */
            if (ISOTP_RECEIVE_STATUS_INPROGRESS != link->receive_status) {
                link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_UNEXP_PDU;
                break;
            }

            /* handle message */
            ret = isotp_receive_consecutive_frame(link, &message, len);

            /* if wrong sn */
            if (ISOTP_RET_WRONG_SN == ret) {
                link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_WRONG_SN;
                link->receive_status = ISOTP_RECEIVE_STATUS_IDLE;
                break;
            }

            /* if success */
            if (ISOTP_RET_OK == ret) {
                /* refresh timer cs */
                link->receive_timer_cr = isotp_user_get_us() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US;
                
                /* receive finished */
                if (link->receive_offset >= link->receive_size) {
                    link->receive_status = ISOTP_RECEIVE_STATUS_FULL;
                } else {
                    /* send fc when bs reaches limit */
                    if (0 == --link->receive_bs_count) {
                        link->receive_bs_count = ISO_TP_DEFAULT_BLOCK_SIZE;
                        isotp_send_flow_control(link, PCI_FLOW_STATUS_CONTINUE, link->receive_bs_count, ISO_TP_DEFAULT_ST_MIN_US);
                    }
                }
            }
            
            break;
        }
        case ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME:
            /* handle fc frame only when sending in progress  */
            if (ISOTP_SEND_STATUS_INPROGRESS != link->send_status) {
                break;
            }

            /* handle message */
            ret = isotp_receive_flow_control_frame(link, &message, len);
            
            if (ISOTP_RET_OK == ret) {
                /* refresh bs timer */
                link->send_timer_bs = isotp_user_get_us() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US;

                /* overflow */
                if (PCI_FLOW_STATUS_OVERFLOW == message.as.flow_control.FS) {
                    link->send_protocol_result = ISOTP_PROTOCOL_RESULT_BUFFER_OVFLW;
                    link->send_status = ISOTP_SEND_STATUS_ERROR;
                }

                /* wait */
                else if (PCI_FLOW_STATUS_WAIT == message.as.flow_control.FS) {
                    link->send_wtf_count += 1;
                    /* wait exceed allowed count */
                    if (link->send_wtf_count > ISO_TP_MAX_WFT_NUMBER) {
                        link->send_protocol_result = ISOTP_PROTOCOL_RESULT_WFT_OVRN;
                        link->send_status = ISOTP_SEND_STATUS_ERROR;
                    }
                }

                /* permit send */
                else if (PCI_FLOW_STATUS_CONTINUE == message.as.flow_control.FS) {
                    if (0 == message.as.flow_control.BS) {
                        link->send_bs_remain = ISOTP_INVALID_BS;
                    } else {
                        link->send_bs_remain = message.as.flow_control.BS;
                    }
                    uint32_t message_st_min_us = isotp_st_min_to_us(message.as.flow_control.STmin);
                    link->send_st_min_us = message_st_min_us > ISO_TP_DEFAULT_ST_MIN_US ? message_st_min_us : ISO_TP_DEFAULT_ST_MIN_US; // prefer as much st_min as possible for stability?
                    link->send_wtf_count = 0;
                }
            }
            break;
        default:
            break;
    };
    
    return needStartTimer;
}

int isotp_receive(IsoTpLink *link, uint8_t *payload, const uint16_t payload_size, uint16_t *out_size) {
    uint16_t copylen;
    
    if (ISOTP_RECEIVE_STATUS_FULL != link->receive_status) {
        return ISOTP_RET_NO_DATA;
    }

    copylen = link->receive_size;
    if (copylen > payload_size) {
        copylen = payload_size;
    }

    memcpy(payload, link->receive_buffer, copylen);
    *out_size = copylen;

    link->receive_status = ISOTP_RECEIVE_STATUS_IDLE;

    return ISOTP_RET_OK;
}

void isotp_init_link(IsoTpLink *link, uint16_t send_arbitration_id, uint16_t receive_arbitration_id) {
    memset(link, 0, sizeof(*link));
    link->receive_status = ISOTP_RECEIVE_STATUS_IDLE;
    link->send_status = ISOTP_SEND_STATUS_IDLE;
    link->send_arbitration_id = send_arbitration_id;
    link->receive_arbitration_id = receive_arbitration_id;
    
    return;
}

void isotp_config_sendbuf(IsoTpLink* link, uint8_t *sendbuf, uint16_t sendbufsize) {
    link->send_buffer = sendbuf;
    link->send_buf_size = sendbufsize;
}

void isotp_config_rcvbuf(IsoTpLink* link, uint8_t *recvbuf, uint16_t recvbufsize) {
    link->receive_buffer = recvbuf;
    link->receive_buf_size = recvbufsize;
}

int isotp_poll(IsoTpLink *link) {
    int ret;
    int stopSendTimer = 0, stopReceiveTimer = 1; /* If need to stop the periodic polling timer */

    /* only polling when operation in progress */
    if (ISOTP_SEND_STATUS_INPROGRESS == link->send_status) {

        /* continue send data */
        if (/* send data if bs_remain is invalid or bs_remain large than zero */
        (ISOTP_INVALID_BS == link->send_bs_remain || link->send_bs_remain > 0) &&
        /* and if st_min is zero or go beyond interval time */
        (0 == link->send_st_min_us || IsoTpTimeAfter(isotp_user_get_us(), link->send_timer_st))) {
            
            ret = isotp_send_consecutive_frame(link);
            if (ISOTP_RET_OK == ret) {
                if (ISOTP_INVALID_BS != link->send_bs_remain) {
                    link->send_bs_remain -= 1;
                }
                link->send_timer_bs = isotp_user_get_us() + ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US;
                link->send_timer_st = isotp_user_get_us() + link->send_st_min_us;

                /* check if send finish */
                if (link->send_offset >= link->send_size) {
                    link->send_status = ISOTP_SEND_STATUS_IDLE;
                }
            } else if (ISOTP_RET_NOSPACE == ret) {
                /* shim reported that it isn't able to send a frame at present, retry on next call */
            } else {
                link->send_status = ISOTP_SEND_STATUS_ERROR;
            }
        }

        /* check timeout */
        if (IsoTpTimeAfter(isotp_user_get_us(), link->send_timer_bs)) {
            link->send_protocol_result = ISOTP_PROTOCOL_RESULT_TIMEOUT_BS;
            link->send_status = ISOTP_SEND_STATUS_ERROR;
        }
    } else {
        /* ERROR or IDLE status, should stop polling timer in either case */

        if (ISOTP_SEND_STATUS_ERROR == link->send_status) {
            /* Reset send status to IDLE so can do next send */
            link->send_status = ISOTP_SEND_STATUS_IDLE;
        }
        stopSendTimer = 1;
    }

    /* only polling when operation in progress */
    if (ISOTP_RECEIVE_STATUS_INPROGRESS == link->receive_status) {
        
        /* check timeout */
        if (IsoTpTimeAfter(isotp_user_get_us(), link->receive_timer_cr)) {
            link->receive_protocol_result = ISOTP_PROTOCOL_RESULT_TIMEOUT_CR;
            link->receive_status = ISOTP_RECEIVE_STATUS_IDLE;
        } else {
            stopReceiveTimer = 0;
        }
    }

    return stopSendTimer & stopReceiveTimer;
}

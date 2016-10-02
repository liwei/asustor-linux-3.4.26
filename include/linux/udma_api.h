#ifndef UDMA_API_H
#define UDMA_API_H

#include <linux/netdevice.h>

/*There're two UDMA ports belong to APPCPU */
#define UDMA_PORT_NUM_TOTAL 2



typedef enum {
        UDMA_OK            = 0x0, /**< 0x0 */
		UDMA_BUSY				, /* UDMA is busy to response */
		UDMA_ERR				, /* UDMA error */
		UDMA_FULL 				, /* The input queue is full to receive new buffers */
		UDMA_EMPTY				, /* The input queue is full to receive new buffers */
		UDMA_INVALID_PARAM			, /* Invalid param */
		UDMA_UNINITIALIZED			, /* UDMA uninitialized */
		UDMA_NO_PERM				, /* UDMA is or going to be stopped that no permision for UDMA access */
		UDMA_AGAIN				 /* Try again */
} udma_result_t;

typedef void (*rx_callback_t)(struct sk_buff *skb, struct net_device *netdev);

/**  udma_send_packet - A buffer is used to send
 * @port - udma port number, could be 0 or 1
 * @buffer_desc - parameter to describe a coming buffer
 *
 * return 0 on success, UDMA driver took care of the buffer
 * return others for failure
*/
int udma_send_packet(unsigned char port, struct sk_buff *skb);




int udma_register_handler(unsigned char port, struct net_device *dev, rx_callback_t rx_handle);

/**  udma_flush - Flush the pending requests in a UDMA port and recycling all of the buffers
 *
 * @port - udma port number, could be 0 or 1
 *
*/
void udma_flush(unsigned char  port);



#endif

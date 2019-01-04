#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/kthread.h>
#include <linux/delay.h>

#include <mach/irqs.h>

#include <linux/semaphore.h>
#include <linux/spinlock.h>

extern void ka2000_enable_irq(unsigned int irq);
extern void ka2000_disable_irq(unsigned int irq);
extern void ka2000_ack_irq(unsigned int irq);

#define NUM_ENDPOINTS		3
#define EP_MAX_PACKET_SIZE	0x200
#define EP0_MAX_PACKET_SIZE	64
#define QH_MAXNUM		32

/*-------------------------------------------------------------*/

#define IO_OFFSET       0x55000000
#define __IO_ADDRESS(x) ((x) + IO_OFFSET)

#define IO_ADDRESS(pa)          IOMEM(__IO_ADDRESS(pa))

#ifdef IOMEM	/* Override asm/io.h */
#undef IOMEM
#endif		/* IOMEM */
#ifdef __ASSEMBLER__
#define IOMEM(x)                x
#else
#define IOMEM(x)                ((void __force __iomem *)(x))
#endif

#define ka2000_readb(a) __raw_readb(IO_ADDRESS(a))
#define ka2000_readw(a) __raw_readw(IO_ADDRESS(a))
#define ka2000_readl(a) __raw_readl(IO_ADDRESS(a))

#define ka2000_writeb(v, a)     __raw_writeb(v, IO_ADDRESS(a))
#define ka2000_writew(v, a)     __raw_writew(v, IO_ADDRESS(a))
#define ka2000_writel(v, a)     __raw_writel(v, IO_ADDRESS(a))

#define W32(a, v)       __raw_writel(v, IO_ADDRESS(a))
#define R32(a)          __raw_readl(IO_ADDRESS(a))

/*-------------------------------------------------------------*/

static int myirq = 0;
static volatile int need_delay = 0;

wait_queue_t wait;
wait_queue_head_t short_queue;

static const char *reqname(unsigned r)
{
	switch (r) {
	case USB_REQ_GET_STATUS: return "GET_STATUS";
	case USB_REQ_CLEAR_FEATURE: return "CLEAR_FEATURE";
	case USB_REQ_SET_FEATURE: return "SET_FEATURE";
	case USB_REQ_SET_ADDRESS: return "SET_ADDRESS";
	case USB_REQ_GET_DESCRIPTOR: return "GET_DESCRIPTOR";
	case USB_REQ_SET_DESCRIPTOR: return "SET_DESCRIPTOR";
	case USB_REQ_GET_CONFIGURATION: return "GET_CONFIGURATION";
	case USB_REQ_SET_CONFIGURATION: return "SET_CONFIGURATION";
	case USB_REQ_GET_INTERFACE: return "GET_INTERFACE";
	case USB_REQ_SET_INTERFACE: return "SET_INTERFACE";
	default: return "*UNKNOWN*";
	}
}

static struct usb_endpoint_descriptor ep0_out_desc = {
	.bLength = sizeof(struct usb_endpoint_descriptor),
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0,
	.bmAttributes = USB_ENDPOINT_XFER_CONTROL,
};

static struct usb_endpoint_descriptor ep0_in_desc = {
	.bLength = sizeof(struct usb_endpoint_descriptor),
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_CONTROL,
};

struct kagen2;
struct kagen2_request {
	struct usb_request req;
	struct list_head queue;
	unsigned mapped:1,
		 valid:1;
};

struct kagen2_ep {
	struct usb_ep ep;
	struct kagen2 *dev;
	unsigned long irqs;

	struct usb_request req;
	struct list_head queue;
	const struct usb_endpoint_descriptor *desc;
	unsigned num:8,
		 fifo_size:12,
		 stopped:1,
		 wedged:1,
		 is_in:1,
		 is_iso:1,
		 dma:1,
		 not_empty:1;
};

#define CONFIG_MAX_PKT(n)     ((n) << 16)

#define TERMINATE 1
#define INFO_BYTES(n)         ((n) << 16)
#define INFO_IOC              (1 << 15)
#define INFO_ACTIVE           (1 << 7)
#define INFO_HALTED           (1 << 6)
#define INFO_BUFFER_ERROR     (1 << 5)
#define INFO_TX_ERROR         (1 << 3)

//static struct kagen2_ep ka_ep_g[NUM_ENDPOINTS];

enum SPEED {
	LOWSPEED = 0,
	FULLSPEED = 1,
	HIGHSPEED = 2,
};

enum STATE {
	DEFAULT = 0,
	SUSPENDED
};

/* tasklet */
struct semaphore sem;
spinlock_t usblock;
int mysync = 0;
static struct tasklet_struct tskLet_USB;

static void ep1_out(struct kagen2 * dev);

void tskLetISR_USB(unsigned long data)
{
        unsigned int i;

        //printk("> %s >>> \n", __func__);

	ep1_out((struct kagen2 *)data);
        //ka2000_enable_irq(32);

}
/* end tasklet */

#if 0
static void
handle_ep_complete(struct kagen2_ep *ka_ep_p, struct kagen2_request *req)
{
	int num, in;

//printk("%s %x\n", __func__, ka_ep_p->desc);
	num = ka_ep_p->desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
	in = (ka_ep_p->desc->bEndpointAddress & USB_DIR_IN) != 0;
	if (num == 0)
		ka_ep_p->desc = &ep0_out_desc;

	//printk("ept%d %s complete %x\n", num, in ? "in" : "out", ka_ep_p->req.length);
     
	// call gadget code
//printk("%x %x %x %x\n", &ka_ep_p->ep, &req->req, req->req.complete, ka_ep_p->req.complete);
	req->req.complete(&ka_ep_p->ep, &req->req);
	list_del_init(&req->queue);
	if (num == 0) {
		ka_ep_p->req.length = 0;
		usb_ep_queue(&ka_ep_p->ep, &req->req, 0);
		ka_ep_p->desc = &ep0_in_desc;
	}
}
#endif

/*--------------------------------------------------------------------------*/

static int
	kagen2_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc);
static int kagen2_ep_disable(struct usb_ep *ep);
static int
	kagen2_ep_queue(struct usb_ep *ep,
		struct usb_request *req, gfp_t gfp_flags);
static int kagen2_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req);
static struct usb_request *
	kagen2_ep_alloc_request(struct usb_ep *ep, unsigned int gfp_flags);
static void kagen2_ep_free_request(struct usb_ep *ep, struct usb_request *_req);
static int
	kagen2_get_frame(struct usb_gadget *_gadget);
static int
	kagen2_wakeup(struct usb_gadget *_gadget);
static int
	kagen2_set_selfpowered(struct usb_gadget *_gadget, int value);
static int
	kagen2_pullup(struct usb_gadget *_gadget, int is_on);
static int
	kagen2_start(struct usb_gadget *_gadget,
		struct usb_gadget_driver *driver);
static int
	kagen2_stop(struct usb_gadget *_gadget,
		struct usb_gadget_driver *driver);
static int
	kagen2_set_halt(struct usb_ep *_ep, int value);
static int
        kagen2_set_wedge(struct usb_ep *_ep);

#define DRIVER_DESC "KAgen2 USB Peripheral Controller"

static const char driver_name[] = "kagen2_usb";
static const char driver_vers[] = "2012 december";
static const char driver_desc[] = DRIVER_DESC;

static const char ep0name[] = "ep0";
static const char * const ep_name[] = {
	ep0name,
	"ep1", "ep1",
};

struct kagen2 {
	/* each device provides one gadget, several endpoints */
	struct usb_gadget gadget;
	struct device *dev;
	unsigned short dev_id;

	spinlock_t lock;
	struct kagen2_ep ep[NUM_ENDPOINTS];
	struct usb_gadget_driver *driver;
	unsigned protocol_stall:1,
		 softconnect:1,
		 is_selfpowered:1,
		 wakeup:1,
		 dma_eot_polarity:1,
		 dma_dack_polarity:1,
		 dma_dreq_polarity:1,
		 dma_busy:1;
	u16 chiprev;
	u8 pagesel;

	unsigned int irq;
	unsigned short fifo_mode;

	unsigned short host_busy;

	void __iomem *base_addr;
};

static void
kagen2_done(struct kagen2_ep *ep, struct kagen2_request *req, int status)
{
        struct kagen2 *dev;
        unsigned stopped = ep->stopped;

        if (ep->num == 0) {
		if (ep->dev->protocol_stall) {
                        kagen2_set_halt(ep, 1);
                }
                ep->stopped = 1;
        }

        list_del_init(&req->queue);

        if (req->req.status == -EINPROGRESS)
                req->req.status = status;
        else
                status = req->req.status;

        dev = ep->dev;
        if (ep->dma)
                usb_gadget_unmap_request(&(dev->gadget), &req->req,
                                ep->is_in);

        if (status && status != -ESHUTDOWN)
                printk("complete %s req %p stat %d len %u/%u buf %p\n",
                        ep->ep.name, &req->req, status,
                        req->req.actual, req->req.length, req->req.buf);

        /* don't modify queue heads during completion callback */
        ep->stopped = 1;
        spin_unlock(&dev->lock);
        req->req.complete(&ep->ep, &req->req);
        spin_lock(&dev->lock);
        ep->stopped = stopped;
}

static void ep0_in(unsigned int phys, int len, struct kagen2 * dev);

#define SETUP(type, request) (((type) << 8) | (request))

static void
handle_setup(struct kagen2 *dev, int bmRequestType, int bRequest,
	int wValue, int wIndex, int wLength)
{
	struct usb_request *req = &(dev->ep[0].req);
	struct usb_ctrlrequest r;
	int status = 0;
	int num, in, _num, _in, i;
	char *buf;
	unsigned char valb;
	unsigned int val;

	if (req == NULL)
	{
		printk("NULL req pointer\n");
		return;
	}

//if (wLength == 0 && wValue == 0 && bRequest != 0x09)
//  return;

	r.bRequestType = bmRequestType;
	r.bRequest = bRequest;
	r.wValue = wValue;
	r.wIndex = wIndex;
	r.wLength = wLength;
	//printk("handle setup %s, %x, %x index %x value %x len %x\n",
	//	reqname(r.bRequest), r.bRequestType, r.bRequest,
	//	r.wIndex, r.wValue, r.wLength);

	switch (SETUP(r.bRequestType, r.bRequest)) {
	case SETUP(USB_RECIP_ENDPOINT, USB_REQ_CLEAR_FEATURE):
printk("USB_RECIP_ENDPOINT\n");
		_num = r.wIndex & 15;
		_in = !!(r.wIndex & 0x80);

		if ((r.wValue == 0) && (r.wLength == 0)) {
			req->length = 0;
			for (i = 0; i < NUM_ENDPOINTS; i++) {
				if (!dev->ep[i].desc)
					continue;
				num = dev->ep[i].desc->bEndpointAddress
					& USB_ENDPOINT_NUMBER_MASK;
				in = (dev->ep[i].desc->bEndpointAddress
					& USB_DIR_IN) != 0;

				if ((num == _num) && (in == _in)) {
					/*ep_enable(num, in);*/
					usb_ep_queue(dev->gadget.ep0, req, 0);
					break;
				}
			}
		}
		return;
	case SETUP(USB_RECIP_DEVICE, USB_REQ_SET_FEATURE):
	case SETUP(USB_RECIP_INTERFACE, USB_REQ_SET_FEATURE):
	case SETUP(USB_RECIP_ENDPOINT, USB_REQ_SET_FEATURE):
printk("USB_REQ_SET_FEATURE 0x%x\n");
		return;	
	case SETUP(USB_RECIP_DEVICE, USB_REQ_SET_ADDRESS):
		/*
		 * write address delayed (will take effect
		 * after the next IN txn)
		 */
printk("USB_REQ_SET_ADDRESS 0x%x\n", wValue);
		req->length = 0;
		//read function address reg
		valb = readb(dev->base_addr + 0x1A6);
		printk("fa is 0x%x\n", valb);
		//usb_ep_queue(dev->gadget.ep0, req, 0);
		return;
	case SETUP(USB_DIR_IN | USB_RECIP_DEVICE, USB_REQ_GET_STATUS):
	case SETUP(USB_DIR_IN | USB_RECIP_INTERFACE, USB_REQ_GET_STATUS):
	case SETUP(USB_DIR_IN | USB_RECIP_ENDPOINT, USB_REQ_GET_STATUS):
printk("USB_REQ_GET_STATUS\n");
		unsigned short status;
		status = 1 << USB_DEVICE_SELF_POWERED;
		ep0_in(&status, 2, dev);
		
#if 0
		req->length = 2;
		buf = (char *)req->buf;
		if (buf == NULL || dev->gadget.ep0 == NULL)
		{
			printk("NULL buf or ep0 pointer\n");
			return;
		}
		buf[0] = 1 << USB_DEVICE_SELF_POWERED;
		buf[1] = 0;
		usb_ep_queue(dev->gadget.ep0, req, 0);
#endif
		return;
	case SETUP(USB_RECIP_DEVICE, USB_REQ_SET_CONFIGURATION):
		printk("USB_REQ_SET_CONFIGURATION\n");
		need_delay = 1;
		break;
	}
	/* pass request up to the gadget driver */
	if (!dev->driver)
	{
//printk("driver not defined\n");
		return;
	}
	if (!dev->driver->setup)
	{
//printk("setup not defined\n");
		return;
	}
	status = dev->driver->setup(&dev->gadget, &r);
        if (status < 0)
        {
                printk("%s status %d\n", __func__, status);
                dev->protocol_stall = 1;
        }
	else if (status == (256+999))
	{
		/*NAK the IN packet from host, and stall endpoint 0*/
		//writeb(readb(dev->base_addr + 0x002) | 0x12, dev->base_addr + 0x002);

//printk("and stall endpoint\n");
                //val = readl(dev->base_addr + 0x000);
                //val &= 0xff00ffff;
                //val |= 0x00010000;
                //writel(val, dev->base_addr + 0x000);

		// toggle reset
		//writeb(0x11, dev->base_addr + 0x1A2);
		//writeb(0x71, dev->base_addr + 0x1A2);
		//writeb(0x01, dev->base_addr + 0x1A2);
		//writeb(0x61, dev->base_addr + 0x1A2);
	}
        return;
}

static void ep1_out(struct kagen2 * dev)
{
	unsigned int val_arr[128];  //SCSI write 10 command has 512 byte data
	int len;
	int i, in, num;
	unsigned int val;
	static unsigned int store_buffer[8];

        //EP1 OUT IRQ
#if 0
        //clear out fifo 0 to 7 empty irq
        val = readl(dev->base_addr + 0x190);
        val &= 0xff00ffff;
        val |= 0x00020000;
        writel(val, dev->base_addr + 0x190);

        // clear Out 0 to 7 irq
        val = readl(dev->base_addr + 0x188);
        val &= 0xff00ffff;
        val |= 0x00020000;
        writel(val, dev->base_addr + 0x188);
#endif

//printk("queue2 0x%x\n", &dev->ep[2].queue);

#if 0
        //bulk out ep
        if (dev->ep[2].desc) {

                struct kagen2_request * ka_req;
                struct usb_request * req;
		int i = 0;
		int loop_cnt = 0;

		// sync with queue function
                //down_interruptible(&sem);

                while (list_empty(&dev->ep[2].queue)) {
                          printk(KERN_EMERG
                           "%s: RX DMA done : NULL REQ on OUT EP-1\n",
                          __func__);
                          return;
			  //msleep(1);
                }

                ka_req = list_entry(dev->ep[2].queue.next, struct kagen2_request, queue);
		//ka_req = container_of(req, struct kagen2_request, req);
                if (ka_req == NULL) {
			printk(KERN_EMERG "%s: no ka_req\n", __func__);
                        return;
		}
		//return;
		//else
			//printk("ka_req Q 0x%x EP %x\n", &ka_req->queue, &dev->ep[2]);

		ka_req->req.actual = 0;

	 	do {
		        // get byte cnt
	        	val = readl(dev->base_addr + 0x008);
	        	len = val & 0xFFF;

	          	// read from fifo1 data
	        	for (i = 0; i < len/4; i++)
		        {
				*((unsigned *)(ka_req->req.buf) + i + (loop_cnt*512/4)) = readl(dev->base_addr + 0x084);
		        }
			if ((len%4) != 0)
			{				
				*((unsigned *)(ka_req->req.buf) + i + (loop_cnt*512/4)) = readl(dev->base_addr + 0x084);
			}
		
	                //ka_req->req.length = len;
        	        //memcpy((((unsigned *)(ka_req->req.buf)) + ka_req->req.actual), &val_arr[0], len);

	                ka_req->req.actual += len;
			loop_cnt++;

	                num = dev->ep[2].desc->bEndpointAddress
                                        & USB_ENDPOINT_NUMBER_MASK;
        	        in = (dev->ep[2].desc->bEndpointAddress
                                        & USB_DIR_IN) != 0;

	                // OUT1CS
        	        val = readl(dev->base_addr + 0x008);
                	val &= 0x00ffffff;
	                writel(val, dev->base_addr + 0x008);

printk(KERN_EMERG "epnum %d in %d len %d %d %d\n", num, in, ka_req->req.actual, ka_req->req.length, len);

		} while ((ka_req->req.actual < ka_req->req.length) && (len != 31) && (ka_req->req.length != 512));

                //printk(KERN_DEBUG "epnum %d in %d len %d %d %d\n", num, in, ka_req->req.actual, ka_req->req.length, len);

printk(KERN_EMERG "%s %x %x\n", __func__, *((char *)(ka_req->req.buf)), *((char *)(ka_req->req.context)));

                handle_ep_complete(&dev->ep[2], ka_req);
	}
	else
#endif
	{
	        // clear the busy bit
        	/*dev->host_busy = 0;
		wake_up_interruptible(&short_queue);*/
	}
//printk("exit ep1_out\n");
}

static irqreturn_t kagen2_irq(int irq, void * _dev)
{

	unsigned int val;
	int i, in, num;
	struct kagen2 *dev = _dev;

	if (dev == NULL)
	{
		printk(KERN_EMERG "NULL dev pointer\n");
		return IRQ_HANDLED;
	}
	spin_lock(&dev->lock);

	ka2000_writel(0x0, 0xa0006030);
//printk("in irq %d\n", irq);
	//ka2000_disable_irq(irq);
	//ka2000_ack_irq(irq);

	// check the IVECT register
#if 1
        val = readl(dev->base_addr + 0x1a0);
	val = val & 0xFF;
//printk("read IVECT 0x%x\n", val);
	if (val == 0xd8)
	{
                if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
	                // set speed to full
        	        dev->gadget.speed = USB_SPEED_FULL;
        	        //dev->gadget.speed = USB_SPEED_HIGH;
			
                        printk(KERN_EMERG "speed is %s\n",
                                usb_speed_string(dev->gadget.speed));
                }

		//peripirq
		val = readl(dev->base_addr + 0x1bc);
		val &= 0xffffff00;
		val |= 0x00000011;		
		writel(val, dev->base_addr + 0x1bc);
	}
	else if (val == 0x10)
	{
		// set speed to unknown
		dev->gadget.speed = USB_SPEED_UNKNOWN;

		// usb reset
		val = readl(dev->base_addr + 0x18c);
		val &= 0xffffff00;
		val |= 0x00000010;		
		writel(val, dev->base_addr + 0x18c);

		// call gadget reset
		if (dev->driver->disconnect)
                	(dev->driver->disconnect)(&dev->gadget);
	}
	else if (val == 0x14)
	{
                // set speed to high
                dev->gadget.speed = USB_SPEED_HIGH;

                printk(KERN_EMERG "speed is %s\n",
                        usb_speed_string(dev->gadget.speed));

		// usb hsirq
		val = readl(dev->base_addr + 0x18c);
		val &= 0xffffff00;
		val |= 0x00000020;		
		writel(val, dev->base_addr + 0x18c);
	}
	else if (val == 0x0C)
	{
		// suspend
		val = readl(dev->base_addr + 0x18c);
		val &= 0xffffff00;
		val |= 0x00000008;		
		writel(val, dev->base_addr + 0x18c);
	}
	else if (val == 0x08)
	{
		// setup token
		val = readl(dev->base_addr + 0x18c);
		val &= 0xffffff00;
		val |= 0x00000004;		
		writel(val, dev->base_addr + 0x18c);
	}
	else if (val == 0x00)
	{
		int bmRequestType;
	        int bRequest;
		unsigned int wLength, wValue, wIndex;
		unsigned int rdata, rdata1;

//if (need_delay == 1)
//{
//	spin_unlock(&dev->lock);

//	return IRQ_HANDLED;
//}
	
	        // setup data valid
                val = readl(dev->base_addr + 0x18c);
	        val &= 0xffffff00;
	        val |= 0x00000001;
                writel(val, dev->base_addr + 0x18c);

		// process the setup data
        	rdata = readl(dev->base_addr + 0x180) ;
	        rdata1 = readl(dev->base_addr + 0x184);

	        bmRequestType = rdata & 0xff;
        	bRequest      = (rdata >> 8) & 0xff;
                wValue        = (rdata >> 16) & 0xffff;
	        wIndex        = rdata1 & 0xffff;
        	wLength       = (rdata1 >> 16) & 0xffff;

	        if (bmRequestType==0x00 && bRequest==0x09)
        	{
                        //clear IVECT reg
                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00110000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00710000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00010000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00610000;
                        writel(val, dev->base_addr + 0x1a0);

                        //handshake stage
                        val = readl(dev->base_addr);
                        val &= 0xff00ffff;
                        val |= 0x00120000;
                        writel(val, dev->base_addr);
                }
		handle_setup(dev, bmRequestType, bRequest, wValue, wIndex, wLength);
	}
	else if (val == 0x1c)
	{
		// EP0OUT TOKEN
		val = readl(dev->base_addr + 0x188);
		val &= 0xff00ffff;
		val |= 0x00010000;		
		writel(val, dev->base_addr + 0x188);
	}
	else if (val == 0x18)
	{
	    if (need_delay == 1)
            {
                // send NAK in respond to IN token
printk("and NAK endpoint\n");
                val = readl(dev->base_addr + 0x000);
                val &= 0xff00ffff;
                val |= 0x00020000;
                writel(val, dev->base_addr + 0x000);
            }
	    else 
	    {
		// EP0IN TOKEN
		val = readl(dev->base_addr + 0x188);
		val &= 0xffffff00;
		val |= 0x00000001;		
		writel(val, dev->base_addr + 0x188);
	    }
	}
	else if (val == 0x2c)
	{
		// EP1 OUT PING
		val = readl(dev->base_addr + 0x18c);
		val &= 0xff00ffff;
		val |= 0x00020000;		
		writel(val, dev->base_addr + 0x18c);
	}
	else if (val == 0x28)
	{
printk(KERN_DEBUG "EP1 OUT IRQ 0x%x\n", val);
		//ep1_out(dev);
		//tskLet_USB.data = (unsigned long)dev;
		//tasklet_schedule(&tskLet_USB);

		ep1_out(dev);
#if 0
        //EP1 OUT IRQ
        //clear out fifo 0 to 7 empty irq
        val = readl(dev->base_addr + 0x190);
        val &= 0xff00ffff;
        val |= 0x00020000;
        writel(val, dev->base_addr + 0x190);

        // clear Out 0 to 7 irq
        val = readl(dev->base_addr + 0x188);
        val &= 0xff00ffff;
        val |= 0x00020000;
        writel(val, dev->base_addr + 0x188);

        // OUT1CS
        val = readl(dev->base_addr + 0x008);
        val &= 0x00ffffff;
        writel(val, dev->base_addr + 0x008);
#endif

		//val = readl(dev->base_addr + 0x1a0);	
	}
	else if (val == 0x24)
	{
		//EP1 in IRQ
		val = readl(dev->base_addr + 0x188);
		val &= 0xffffff00;
		val |= 0x00000002;		
		writel(val, dev->base_addr + 0x188);
		
	}
	else if (val == 0x20)
	{
		//EP0 PING
	}
	else
	{
		printk("unknown 0x%x\n", val);
	}
#endif
	//ka2000_enable_irq(irq);

	spin_unlock(&dev->lock);

	return IRQ_HANDLED;
}

static void kagen2_lowlevel_init(struct kagen2 * dev)
{
	unsigned int val;

	// unmask the irq 32 which is the usb irq
	//ka2000_writel((ka2000_readl(0xa0006014) & 0xfffffffe), 0xa0006014);
	//ka2000_writel((0xfffffffe), 0xa0006014);

	//read pclk
	printk("read pclk %x scu %x irqmask %x %x\n", ka2000_readl(0xa0000028), 
		ka2000_readl(0xa0000094), ka2000_readl(0xa0006010), ka2000_readl(0xa0006014));

        //set CONF2 = 1
        val = R32(0xa000003c);
        printk("val is 0x%x\n", val);
        W32(0xa000003C, 0x10);
        val = R32(0xa000003c);
        printk("val is 0x%x\n", val);

loopsta:
	// IVECT
	val = readl(dev->base_addr + 0x1a0);
	val &= 0x00ffffff;
	writel(val, dev->base_addr + 0x1a0);

	// INXMAXPCKL - 0x3e2 INXMAXPCKH - 0x3e3
	val = readl(dev->base_addr + 0x3e0);
	val &= 0x0000ffff;
	val |= 0x02000000;
	writel(val, dev->base_addr + 0x3e0);

	// OUTXMAXPCKL - 0x1e2 OUTXMAXPCKH - 0x1e3
	val = readl(dev->base_addr + 0x1e0);
	val &= 0x0000ffff;
	val |= 0x02000000;
	writel(val, dev->base_addr + 0x1e0);

	// INXSTARTADDRL - 0x344 INXSTARTADDRH - 0x345
	val = readl(dev->base_addr + 0x344);
	val &= 0xffff0000;
	val |= 0x00000040;
	writel(val, dev->base_addr + 0x344);

	// OUTxSTARTADDRL - 0x304 OUTxSTARTADDRH - 0x305
	val = readl(dev->base_addr + 0x304);
	val &= 0xffff0000;
	val |= 0x00000240;
	writel(val, dev->base_addr + 0x304);
	
	// IN1CON - 0x00e
	val = readl(dev->base_addr + 0x00C);
	val &= 0xff00ffff;
	val |= 0x00880000;
	writel(val, dev->base_addr + 0x00C);

	// IN2CON - 0x016
	val = readl(dev->base_addr + 0x014);
	val &= 0xff00ffff;
	val |= 0x00880000;
	writel(val, dev->base_addr + 0x014);

	// OUT1CON - 0x00A
	val = readl(dev->base_addr + 0x008);
	val &= 0xff00ffff;
	val |= 0x00880000;
	writel(val, dev->base_addr + 0x008);

	// OUT2CON - 0x012
	val = readl(dev->base_addr + 0x010);
	val &= 0xff00ffff;
	val |= 0x00880000;
	writel(val, dev->base_addr + 0x010);

	// FIFOCTRL IN/OUT -0x1a8 for EP1
	val = readl(dev->base_addr + 0x1a8);
	val &= 0xffffff00;
	val |= 0x00000051;
	writel(val, dev->base_addr + 0x1a8);

	val = readl(dev->base_addr + 0x1a8);
	val &= 0xffffff00;
	val |= 0x00000001;
	writel(val, dev->base_addr + 0x1a8);

	// FIFOCTRL IN/OUT -0x1a8 for EP2
	val = readl(dev->base_addr + 0x1a8);
	val &= 0xffffff00;
	val |= 0x00000052;
	writel(val, dev->base_addr + 0x1a8);

	val = readl(dev->base_addr + 0x1a8);
	val &= 0xffffff00;
	val |= 0x00000002;
	writel(val, dev->base_addr + 0x1a8);

	// FIFORST togglerst - 0x1a0 for IN EP1
	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00110000;
	writel(val, dev->base_addr + 0x1a0);

	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00710000;
	writel(val, dev->base_addr + 0x1a0);

	// FIFORST togglerst - 0x1a0 for IN EP2
	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00120000;
	writel(val, dev->base_addr + 0x1a0);

	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00720000;
	writel(val, dev->base_addr + 0x1a0);

	// FIFORST togglerst - 0x1a0 for OUT EP1
	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00010000;
	writel(val, dev->base_addr + 0x1a0);

	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00610000;
	writel(val, dev->base_addr + 0x1a0);
	
	// FIFORST togglerst - 0x1a0 for OUT EP2
	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00020000;
	writel(val, dev->base_addr + 0x1a0);

	val = readl(dev->base_addr + 0x1a0);
	val &= 0xff00ffff;
	val |= 0x00620000;
	writel(val, dev->base_addr + 0x1a0);

	// INxIEN - 0x194 
	val = readl(dev->base_addr + 0x194);
	val &= 0xffffff00;
	val |= 0x00000004;
	writel(val, dev->base_addr + 0x194);

	// INxFULLIEN - 0x19C
	val = readl(dev->base_addr + 0x19C);
	val &= 0xffffff00;
	val |= 0x00000004;
	writel(val, dev->base_addr + 0x19C);

	// OUTxIEN 0x196
	val = readl(dev->base_addr + 0x194);
	val &= 0xff00ffff;
	val |= 0x00040000;
	writel(val, dev->base_addr + 0x194);

	// OUTxEMPTIEN 0x19E
	val = readl(dev->base_addr + 0x19C);
	val &= 0xff00ffff;
	val |= 0x00040000;
	writel(val, dev->base_addr + 0x19C);

	// OTG periph irq
	val = readl(dev->base_addr + 0x1C0);
	val &= 0xffffff00;
	val |= 0x00000010;
	writel(val, dev->base_addr + 0x1C0);

#if 0
{
	int loop = 0;
	val = readl(dev->base_addr + 0x1a0);
	printk("check USB_IVECT 0x%x \n", val);
	while ((val&0xFF) !=0xD8) {
		val = readl(dev->base_addr + 0x1bc);
		printk("check USB_OTGST 0x%x \n", val);

		val = readl(dev->base_addr + 0x1a0);
		printk("check USB_IVECT 0x%x \n", val);

		if (loop++ > 50)
		{
		  printk("reset USB\n");
		  // usb reset 1
		  ka2000_writel((ka2000_readl(0xa000003c) | 0x1), 0xa000003c);
		  // usb reset 0
		  ka2000_writel((ka2000_readl(0xa000003c) & 0xfffffffe), 0xa000003c);

		  // usb reset 0
		  ka2000_writel((ka2000_readl(0xa000002c) & 0xfffffffe), 0xa000002c);
		  // usb reset 1
		  ka2000_writel((ka2000_readl(0xa000002c) | 0x1), 0xa000002c);
		  goto loopsta;
		}
	};
}
#endif
val = readl(dev->base_addr + 0x1bc);
printk("check USB_OTGST 0x%x \n", val);

	// OTGIRQ
	val = readl(dev->base_addr + 0x1bc);
	val &= 0xffffff00;
	val |= 0x00000010;
	writel(val, dev->base_addr + 0x1bc);
printk("check USB_OTGIRQ 0x%x \n", val);
	
	// intr init
	val = readl(dev->base_addr + 0x198);
	val &= 0xff00ff00;
	val |= 0x000700fd;
	writel(val, dev->base_addr + 0x198);

	val = readl(dev->base_addr + 0x194);
	val &= 0xff00ff00;
	val |= 0x00070007;
	writel(val, dev->base_addr + 0x194);
printk("check USB_IRQINIT 0x%x \n", val);

	//OTGFSM WAKEUPDP
	val = readl(dev->base_addr + 0x1c0);
	val &= 0xffffff00;
	val |= 0x0000001f;
	writel(val, dev->base_addr + 0x1c0);
printk("check USB_OTGFSM 0x%x \n", val);

	// OTGCTRL
	val = readl(dev->base_addr + 0x1bc);
	val &= 0xff00ffff;
	val |= 0x00300000;
	writel(val, dev->base_addr + 0x1bc);
printk("check USB_OTGCTRL 0x%x \n", val);

	// set usb irq 32 to edge trigger
	ka2000_writel(0x1, 0xa000602c);
	// clear irq
	ka2000_writel(0x0, 0xa0006030);

	if (myirq == 1)
	{
		printk("myirq is %d\n", myirq);	
		ka2000_writel((0xfffffffe), 0xa0006014);
	}
}

static struct usb_ep_ops kagen2_ep_ops = {
        .enable        = kagen2_ep_enable,
        .disable       = kagen2_ep_disable,

        .alloc_request = kagen2_ep_alloc_request,
        .free_request  = kagen2_ep_free_request,

	.queue         = kagen2_ep_queue,
	.dequeue       = kagen2_ep_dequeue,

	.set_halt      = kagen2_set_halt,
        .set_wedge     = kagen2_set_wedge,
};

static struct usb_gadget_ops kagen2_gadget_ops = {
        .get_frame              = kagen2_get_frame,
        .wakeup                 = kagen2_wakeup,
        .set_selfpowered        = kagen2_set_selfpowered,
        .pullup                 = kagen2_pullup,
        .udc_start              = kagen2_start,
        .udc_stop               = kagen2_stop,
};

static int
kagen2_get_frame(struct usb_gadget *_gadget)
{
        struct kagen2 *dev;
        unsigned long flags;
        u16 ret;

        if (!_gadget)
                return -ENODEV;
        dev = container_of(_gadget, struct kagen2, gadget);
        spin_lock_irqsave(&dev->lock, flags);

        spin_unlock_irqrestore(&dev->lock, flags);
        return ret;
}

static int
kagen2_wakeup(struct usb_gadget *_gadget)
{
        struct kagen2 *dev;
        unsigned long flags;

        if (!_gadget)
                return 0;
        dev = container_of(_gadget, struct kagen2, gadget);

        spin_lock_irqsave(&dev->lock, flags);

        spin_unlock_irqrestore(&dev->lock, flags);

        return 0;
}

static int
kagen2_set_selfpowered(struct usb_gadget *_gadget, int value)
{
        struct kagen2 *dev;

        if (!_gadget)
                return -ENODEV;
        dev = container_of(_gadget, struct kagen2, gadget);

        dev->is_selfpowered = value;

        return 0;
}

static int
kagen2_pullup(struct usb_gadget *_gadget, int is_on)
{
        struct kagen2 *dev;
        unsigned long flags;

        if (!_gadget)
                return -ENODEV;
        dev = container_of(_gadget, struct kagen2, gadget);

        spin_lock_irqsave(&dev->lock, flags);
        dev->softconnect = (is_on != 0);
        spin_unlock_irqrestore(&dev->lock, flags);

        return 0;
}

static int kagen2_start(struct usb_gadget *_gadget,
                struct usb_gadget_driver *driver)
{
        struct kagen2 *dev;
        unsigned i;
	struct kagen2_ep *ka_ep;

printk("0x%x 0x%x\n", driver, driver->setup);
        //if (!driver || !driver->unbind || !driver->setup ||
        //    driver->max_speed != USB_SPEED_HIGH)
	if (!driver)
                return -EINVAL;

        dev = container_of(_gadget, struct kagen2, gadget);

printk("%s\n", __func__);
printk("0x%x 0x%x\n", driver, driver->setup);

        for (i = 0; i < 4; ++i)
                dev->ep[i].irqs = 0;
        /* hook up the driver ... */
        dev->softconnect = 1;
        driver->driver.bus = NULL;
        dev->driver = driver;
        dev->gadget.dev.driver = &driver->driver;
	dev->gadget.speed = USB_SPEED_FULL;
	//dev->gadget.speed = USB_SPEED_HIGH;

	//kagen2_ep0_start();
	ka_ep = (struct kagen2_ep *)&dev->ep[0];
printk("0x%x 0x%x\n", &ka_ep->ep, &ep0_in_desc);
	kagen2_ep_enable(&ka_ep->ep, &ep0_in_desc);
	//ka_ep = (struct kagen2_ep *)&dev->ep[1];
	//kagen2_ep_enable(&ka_ep->ep, &ep0_out_desc);

	return 0;
}

static int kagen2_stop(struct usb_gadget *_gadget,
                struct usb_gadget_driver *driver)
{
        struct kagen2 *dev;
        unsigned long flags;

        dev = container_of(_gadget, struct kagen2, gadget);

        spin_lock_irqsave(&dev->lock, flags);

        spin_unlock_irqrestore(&dev->lock, flags);

        dev->gadget.dev.driver = NULL;
        dev->driver = NULL;

        return 0;
}

static int kagen2_ep_enable(struct usb_ep *ep,
                const struct usb_endpoint_descriptor *desc)
{
        struct kagen2_ep *ka_ep = container_of(ep, struct kagen2_ep, ep);
        int num, in;

        num = desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
        in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
        ka_ep->desc = desc;

//printk("%s %d %d\n",__func__, num, in);
//printk("%x %x %x \n", ep, desc, ka_ep);
        return 0;
}

static int kagen2_ep_disable(struct usb_ep *ep)
{
        return 0;
}

static struct usb_request *
kagen2_ep_alloc_request(struct usb_ep *ep, unsigned int gfp_flags)
{
        struct kagen2_ep *ka_ep;
	struct kagen2_request *req;

        if (!ep)
                return NULL;
        ka_ep = container_of(ep, struct kagen2_ep, ep);

        req = kzalloc(sizeof(*req), gfp_flags);
        if (!req)
                return NULL;

        INIT_LIST_HEAD(&req->queue);

        return &req->req;
}

static void kagen2_ep_free_request(struct usb_ep *ep, struct usb_request *_req)
{
        return;
}

static void ep0_in(unsigned int phys, int len, struct kagen2 * dev)
{
	unsigned int val32;
	unsigned int iter_num = 0;
	int i;

	// fill IN zero data buffer
	for (i = 0; i < len/4; i++)
	{
		val32 = *((unsigned int *)(phys + (i*4)));
//printk("0x%x\n", val32);
		writel(val32, dev->base_addr + 0x100+(i*4));
	}
	if ((len%4) != 0)
	{
		val32 = *((unsigned int *)(phys + (i*4)));
//printk("Last 0x%x\n", val32);
		writel(val32, dev->base_addr + 0x100+(i*4));
	}

	//udelay(20);
	// write to in0bc, arm the transfer
	val32 = readl(dev->base_addr);
	val32 &= 0xffff00ff;
	val32 |= len << 8;
	writel(val32, dev->base_addr);

        // check for IN0-7 IRQ
        val32 = readl(dev->base_addr + 0x188);
        while ((val32&0xff) != 0x01)
        {
        	val32 = readl(dev->base_addr + 0x188);
             	if (iter_num++ > 140)
                        break;
	
	}
        // clear IN0-7 IRQ
        val32 &= 0xffffff00;
        val32 |= 0x00000001;
        writel(val32, dev->base_addr + 0x188);
                
	//handshake stage
        val32 = readl(dev->base_addr);
        val32 &= 0xff00ffff;
        val32 |= 0x00020000;  //clear HSNAK bit
        writel(val32, dev->base_addr);

	return;
}

static void ep1_in(unsigned int phys, int len, struct kagen2 * dev)
{
        unsigned int iter_num = 0, len_num = 0, loop_num = 0;
        volatile unsigned int val32;
        int i;
        int  num = 1;
	int max_len = 512;
	unsigned  fifo1dat;
	unsigned int buffer;

  //check speed mode
  if (dev->gadget.speed == USB_SPEED_HIGH)
  {
	max_len = 512;
  }
  else 
  {
	max_len = 64;
  }

//printk(KERN_EMERG "max_len %d\n", max_len);

	if (len > max_len)	
	{
		// break down the data into chunks of 512 bytes
		len_num = len;
		do
		{
printk(KERN_DEBUG "len_num %d, iter_num %d\n", len_num, iter_num);

#if 1
			fifo1dat = (unsigned int)dev->base_addr + 0x84;
			buffer = phys + (max_len*iter_num);
			// use inline assembly to copy to FIFO
			__asm__ __volatile__ (
				"mov r3, %[repeat], LSR#2\n"
				"1:\n"
				"ldr r2, [%[input]], #4\n"    
				"str r2, [%[fifo]]\n"
				"sub r3, r3, #1\n"
				"cmp r3, #0\n"
				"bne 1b\n"
				:
				:[fifo]"r"(fifo1dat), [input]"r"(buffer), [repeat]"r"(max_len)
				:"r2", "r3", "memory", "cc"
			);

#endif
#if 0
			// fill IN FIFO x data register
			for (i = 0; i < max_len/4; i++)
		        {
                		val32 = *((unsigned int *)(phys + (max_len*iter_num) + (i*4)));
		                writel(val32, dev->base_addr + 0x84);
if (i < 4)
  printk(KERN_DEBUG "%x: 0x%x ", (i*4), val32);
        		}
#endif

		        // write to inXbc, arm the transfer
		        val32 = readl(dev->base_addr + 0x0C );
		        val32 &= 0xffff0000;
		        val32 |= max_len;
		        writel(val32, dev->base_addr + 0x0C );

		        // IN1CS reg
		        val32 = readl(dev->base_addr + 0x0C );
		        val32 &= 0x00ffffff;
		        writel(val32, dev->base_addr + 0x0C );

			// check for IN0-7 IRQ
		        val32 = readl(dev->base_addr + 0x188);
		        while (((val32&0xff) != 0x2) && ((val32&0xff) != 0x3) )
		        {
		                val32 = readl(dev->base_addr + 0x188);
				printk(KERN_DEBUG "val32 is %x\n", val32);
		                if (loop_num++ > 140)
				{
					loop_num = 0;
					printk(KERN_EMERG "no IN1 IRQ\n");
                			break;
				}
		        }
		        // clear IN0-7 IRQ
		        val32 &= 0xffffff00;
		        val32 |= 0x00000001 << num;
		        writel(val32, dev->base_addr + 0x188);

			len_num -= max_len;
			iter_num++;
		} while (len_num > 0);	
	}
	else
	{
		// less than max_len bytes length
	        // fill IN FIFO x data register
	        for (i = 0; i < len/4; i++)
        	{
        	        val32 = *((unsigned int *)(phys + (i*4)));
if (i < 4)
  printk(KERN_DEBUG "%x: 0x%x\n", (i*4), val32);
	                writel(val32, dev->base_addr + 0x84);
	        }
	        if ((len%4) != 0)
	        {
	                val32 = *((unsigned int *)(phys + (i*4)));
//printk("Last 0x%x\n", val32);
	                writel(val32, dev->base_addr + 0x84);
	        }
	        // write to inXbc, arm the transfer
		val32 = readl(dev->base_addr + 0x0C );
	        val32 &= 0xffff0000;
	        val32 |= len;
	        writel(val32, dev->base_addr + 0x0C );

		// IN1CS reg
	        val32 = readl(dev->base_addr + 0x0C );
	        val32 &= 0x00ffffff;
	        writel(val32, dev->base_addr + 0x0C );

	        /*val32 = readl(dev->base_addr + 0x0C );
	        val32 &= 0xffff00ff;
	        val32 |= len >> 16;
	        writel(val32, dev->base_addr + 0x0C );*/
	
        	// check for IN0-7 IRQ
	        val32 = readl(dev->base_addr + 0x188);
        	while (((val32&0xff) != 0x2) && ((val32&0xff) != 0x3) )
	        {	
        	        val32 = readl(dev->base_addr + 0x188);
			if (iter_num++ > 140)
				break;
        	}
        	// clear IN0-7 IRQ
        	val32 &= 0xffffff00;
        	val32 |= 0x00000001 << num;
        	writel(val32, dev->base_addr + 0x188);
	}
	return;
}

static int kagen2_ep_queue(struct usb_ep *ep,
                struct usb_request *req, gfp_t gfp_flags)
{
        struct kagen2_ep *ka_ep;
	struct kagen2_request *ka_req;
	struct kagen2 * dev;
        unsigned phys;
        int num, len, in;
	unsigned long flags;

//printk("%s\n",__func__);

	ka_req = container_of(req, struct kagen2_request, req);
        if (!req || !req->complete || !req->buf
                        || !list_empty(&ka_req->queue))
	{
		printk("exit A\n");
                return -EINVAL;
	}
	ka_ep = container_of(ep, struct kagen2_ep, ep);
        if (!ep || (!ka_ep->desc && ka_ep->num != 0))
	{
		printk("exit B\n");
                return -EINVAL;
	}
        dev = ka_ep->dev;

//printk("0x%x 0x%x 0x%x 0x%x\n", dev, dev->driver, ka_ep, ka_req);
        if (!dev || !dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
	{
		printk("exit C\n");
		if (dev->gadget.speed == USB_SPEED_UNKNOWN)
			dev->gadget.speed = USB_SPEED_FULL;
		else
                	return -ESHUTDOWN;
	}
        num = ka_ep->desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
        in = (ka_ep->desc->bEndpointAddress & USB_DIR_IN) != 0;
        phys = (unsigned)req->buf;
        len = req->length;

printk(KERN_DEBUG "ept%d %s queue len 0x%x, buffer 0x%x\n",
                        num, in ? "in" : "out", len, phys);

if (num == 0 && len == 0)
{
	unsigned int val;

	need_delay = 0;

                // EP0IN TOKEN
                //val = readl(dev->base_addr + 0x188);
                //val &= 0xffffff00;
                //val |= 0x00000001;
                //writel(val, dev->base_addr + 0x188);
}

	spin_lock_irqsave(&dev->lock, flags);

	if ((len >= 0) && (num == 0) && (in != 0))
	{
		req->actual = 0;
                ep0_in(phys, len, dev);
		req->actual += len;
		spin_unlock(&dev->lock);
		req->complete(ep, req);
		spin_lock(&dev->lock);
		list_del_init(&ka_req->queue);
                goto done;	
	}
	else if ((len >= 0) && (num == 1) && (in != 0))
	{
		req->actual = 0;
                ep1_in(phys, len, dev);
		req->actual += len;
		spin_unlock(&dev->lock);
		req->complete(ep, req);
		spin_lock(&dev->lock);
		list_del_init(&ka_req->queue);
                goto done;	
	}
	else if (in == 0)
    	{
		// read from EPxOUT buffer
	        if (num == 1)
        	{
			unsigned int val;
			unsigned int val_arr[128];  //SCSI write 10 command has 512 byte data
			int i;
			int loop_cnt = 0;
			unsigned char valb;
			int max_len = 512;

  //check speed mode
  if (dev->gadget.speed == USB_SPEED_HIGH)
  {
        max_len = 512;
  }
  else
  {
        max_len = 64;
  }

			//struct usb_request * temp = &(ka_ep->req);
			//printk("EP1 %x\n", (u8 *)ka_ep);

			//list_add_tail(&ka_req->queue, &ka_ep->queue);
			
			//up(&sem);
	                //spin_lock(&usblock);
        	        //mysync = 1;
                	//spin_unlock(&usblock);

			//if (list_empty(&ka_ep->queue))
			//	printk("list is empty\n");

			list_add_tail(&ka_req->queue, &ka_ep->queue);
			req->actual = 0;

			do {			

			// is endpoint busy
			spin_unlock(&dev->lock);
printk(KERN_DEBUG "before %s\n", __func__);
			wait_event_interruptible(short_queue, dev->host_busy == 0);
printk(KERN_DEBUG "after %s\n", __func__);
			spin_lock(&dev->lock);

//if (need_delay == 1)
//	goto done;
			//while (1) {
			//    set_current_state(TASK_INTERRUPTIBLE);
			//    if (dev->host_busy == 0) /* whatever test your driver needs */
			//	    break;
			//    schedule();
			//}
			//set_current_state(TASK_RUNNING);
			dev->host_busy = 1;

	//do
	//{
	//  val = readl(dev->base_addr + 0x008);
	//  val &= 0x02000000;
	//} while (val == 0x02000000);

				// get byte cnt
                                val = readl(dev->base_addr + 0x008);
                                len = val & 0xFFF;
//printk("len %d \n",len);

		        	// read from fifo1 data
	        		for (i = 0; i < len/4; i++)
			        {
        		        	*((unsigned *)(ka_req->req.buf) + i + (loop_cnt*max_len/4)) = readl(dev->base_addr + 0x084);
		        	}
				if ((len%4) != 0)
				{
					*((unsigned *)(ka_req->req.buf) + i + (loop_cnt*max_len/4)) = readl(dev->base_addr + 0x084);
				}

			        //clear out fifo 0 to 7 empty irq
				valb = readb(dev->base_addr + 0x192);
				valb |= 0x02;
				writeb(valb, dev->base_addr + 0x192);

			        //val = readl(dev->base_addr + 0x190);
			        //val &= 0xff00ffff;
			        //val |= 0x00020000;
			        //writel(val, dev->base_addr + 0x190);

			        // clear Out 0 to 7 irq
			        valb = readb(dev->base_addr + 0x18A);
			        valb |= 0x02;
			        writeb(valb, dev->base_addr + 0x18A);

			        //val = readl(dev->base_addr + 0x188);
			        //val &= 0xff00ffff;
			        //val |= 0x00020000;
			        //writel(val, dev->base_addr + 0x188);
				
	                        // OUT1CS
        	                val = readl(dev->base_addr + 0x008);
                	        val &= 0x00ffffff;
	                        writel(val, dev->base_addr + 0x008);

				//list_add_tail(&ka_req->queue, &ka_ep->queue);

				//memcpy((((unsigned char *)ka_req->req.buf) + ka_req->req.actual), &val_arr[0], len);
		
				ka_req->req.actual += len;
				loop_cnt++;
printk(KERN_DEBUG "%s %d %d %d\n", __func__, ka_req->req.actual, ka_req->req.length, len);
			
			} while ((ka_req->req.actual < ka_req->req.length) && (len != 31) && ( ka_req->req.length != max_len));
			
			//handle_ep_complete(ka_ep, ka_req);
			if (len > 0) {
//printk(KERN_EMERG "%s %x %x\n", __func__, *((char *)(ka_req->req.buf)), *((char *)(ka_req->req.context)));
printk(KERN_DEBUG "[%s] %x %x\n", __func__, *((unsigned int *)(ka_req->req.buf)), *((unsigned int *)(ka_req->req.buf + 4)));
				spin_unlock(&dev->lock);
				ka_req->req.complete(&ka_ep->ep, &ka_req->req);
				spin_lock(&dev->lock);
				list_del_init(&ka_req->queue);
			}
		}
    	}

done:
	spin_unlock_irqrestore(&dev->lock, flags);
        return 0;
}

static int
kagen2_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct kagen2_ep *ep;
	struct kagen2_request *req;
	unsigned long flags;

//printk("%s\n",__func__);

	ep = container_of(_ep, struct kagen2_ep, ep);

	spin_lock_irqsave(&ep->dev->lock, flags);

        /* make sure it's still queued on this endpoint */
        list_for_each_entry(req, &ep->queue, queue) {
                if (&req->req == _req)
                        break;
        }
        if (&req->req != _req) {
                spin_unlock_irqrestore(&ep->dev->lock, flags);
                return -EINVAL;
        }
        /* queue head may be partially complete */
        if (ep->queue.next == &req->queue) {
                printk("unlink (%s) pio\n", _ep->name);
                kagen2_done(ep, req, -ECONNRESET);
        }
	req = NULL;

        spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

static int
kagen2_set_halt(struct usb_ep *_ep, int value)
{
        struct kagen2_ep *ep;
        int ret = 0;
        int in;
        struct kagen2 * dev;

        ep = container_of(_ep, struct kagen2_ep, ep);
        if (!_ep || (!ep->desc && ep->num != 0))
                return -EINVAL;
        in = (ep->desc->bEndpointAddress & USB_DIR_IN) != 0;
        dev = ep->dev;

#if 1
printk("%s %d %d\n", __func__, value, in);
        if (value) {
                if (ep->num == 0)
                        ep->dev->protocol_stall = 1;
                else {
                        //set_halt
                        if (in == 1) {
                                //IN1CON
                                writel(0x48, dev->base_addr + 0x0a);
                                writel(0x48, dev->base_addr + 0x0e);
			}
                        else {
                                //OUT1CON
                                writel(0x48, dev->base_addr + 0x0e);
                                writel(0x48, dev->base_addr + 0x0a);
			}
                }
        } else {
                //clear_halt
                if (in == 1)
                        //IN1CON
                        writel(0x08, dev->base_addr + 0x0e);
                else
                        //OUT1CON
                        writel(0x08, dev->base_addr + 0x0a);
                ep->wedged = 0;
        }
#endif
        return ret;
}

static int
kagen2_set_wedge(struct usb_ep *_ep)
{
        struct kagen2_ep *ep;
        int ret = 0;
        int in;
        struct kagen2 * dev;

        if (!_ep || _ep->name == ep0name)
                return -EINVAL;

        ep = container_of(_ep, struct kagen2_ep, ep);
        if (!_ep || (!ep->desc && ep->num != 0))
                return -EINVAL;
        in = (ep->desc->bEndpointAddress & USB_DIR_IN) != 0;
        dev = ep->dev;


        if (1) {
                if (ep->num == 0)
                        ep->dev->protocol_stall = 1;
                else {
                        //set_halt
                        if (in == 1)
                                //IN1CON
                                writel(0x48, dev->base_addr + 0x0e);
                        else
                                //OUT1CON
                                writel(0x48, dev->base_addr + 0x0a);
                }
        }
        ep->wedged = 1;

	return ret;
}

static void
kagen2_gadget_release(struct device *_dev)
{
        struct kagen2 *dev = dev_get_drvdata(_dev);
        kfree(dev);
}

// for kernel thread
struct task_struct * task;

static int chkbusy_thread(void * data)
{
	struct kagen2 * dev = (struct kagen2 *)data;
	unsigned int val;

	if (dev == NULL)
	{
		printk(KERN_EMERG "NULL dev pointer\n");
		return 0;
	}
	for (;;)
	{
	        do
        	{
	          val = readl(dev->base_addr + 0x008);
        	  val &= 0x02000000;
	        } while (val == 0x02000000);

	   //if (need_delay == 0)
	   //{
                // clear the busy bit
                dev->host_busy = 0;

		wake_up_interruptible(&short_queue);
	   //}
	   //else
	   //{
		//printk("need_delay is 1\n");
	   //}
		//msleep(1);
	}
	return 0;
}

#if 0
static int example_thread(void * data)
{
	int i = 0;
	volatile unsigned int val;
	struct kagen2 * dev = (struct kagen2 *)data;

	for (;;)
	{
	    if (++i == 10)
	    {
		i = 0;
		val = readl(dev->base_addr + 0x1a0);
		val = val & 0xff;
//printk("INTC IVECT %x\n", val);		
		if (val == 0xd8)
		{
                	//peripirq
	               	val = readl(dev->base_addr + 0x1bc);
	       	        val &= 0xffffff00;
       	        	val |= 0x00000011;
               		writel(val, dev->base_addr + 0x1bc);
		}
		else if (val == 0x00)
		{
printk("examplethread 0x%x\n", val);
			int bmRequestType;
	                int bRequest;
			unsigned int wLength, wValue, wIndex;
			unsigned int rdata, rdata1;
	
	                // setup data valid
               		val = readl(dev->base_addr + 0x18c);
	                val &= 0xffffff00;
	                val |= 0x00000001;
                	writel(val, dev->base_addr + 0x18c);

			// process the setup data
        	        rdata = readl(dev->base_addr + 0x180) ;
	                rdata1 = readl(dev->base_addr + 0x184);

	                bmRequestType = rdata & 0xff;
        	        bRequest      = (rdata >> 8) & 0xff;
                	wValue        = (rdata >> 16) & 0xffff;
	                wIndex        = rdata1 & 0xffff;
        	        wLength       = (rdata1 >> 16) & 0xffff;

	                if (bmRequestType==0x00 && bRequest==0x09)
        	        {
                        //clear IVECT reg
                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00110000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00710000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00010000;
                        writel(val, dev->base_addr + 0x1a0);

                        val = readl(dev->base_addr + 0x1a0);
                        val &= 0xff00ffff;
                        val |= 0x00610000;
                        writel(val, dev->base_addr + 0x1a0);

                        //handshake stage
                        val = readl(dev->base_addr);
                        val &= 0xff00ffff;
                        val |= 0x00120000;
                        writel(val, dev->base_addr);
                	}
			handle_setup(dev, bmRequestType, bRequest, wValue, wIndex, wLength);
		}
	        else if (val == 0x28)
        	{
printk("EPx OUT IRQ 0x%x\n", val);
			ep1_out(dev);

			val = readl(dev->base_addr + 0x1a0);	
		}
		else if (val == 0x08)
        	{
        	        // setup token
	                val = readl(dev->base_addr + 0x18c);
                	val &= 0xffffff00;
        	        val |= 0x00000004;
	                writel(val, dev->base_addr + 0x18c);
		}
	        else if (val == 0x14)
        	{
	                // usb hsirq
        	        val = readl(dev->base_addr + 0x18c);
                	val &= 0xffffff00;
	                val |= 0x00000020;              
        	        writel(val, dev->base_addr + 0x18c);
	        }
        	else if (val == 0x0C)
	        {
        	        // suspend
                	val = readl(dev->base_addr + 0x18c);
	                val &= 0xffffff00;
        	        val |= 0x00000008;              
                	writel(val, dev->base_addr + 0x18c);
        	}
	        else if (val == 0x1c)
        	{
	                // EP0OUT TOKEN
        	        val = readl(dev->base_addr + 0x188);
                	val &= 0xff00ffff;
	                val |= 0x00010000;
        	        writel(val, dev->base_addr + 0x188);
	        }
        	else if (val == 0x18)
	        {
        	        // EP0IN TOKEN
                	val = readl(dev->base_addr + 0x188);
	                val &= 0xffffff00;
        	        val |= 0x00000001;
                	writel(val, dev->base_addr + 0x188);
	        }
        	else if (val == 0x2c)
	        {
        	        // EP1 OUT PING
                	val = readl(dev->base_addr + 0x18c);
	                val &= 0xff00ffff;
        	        val |= 0x00020000;
                	writel(val, dev->base_addr + 0x18c);
	        }
	        else if (val == 0x10)
        	{
	                // usb reset
        	        val = readl(dev->base_addr + 0x18c);
                	val &= 0xffffff00;
	                val |= 0x00000010;              
        	        writel(val, dev->base_addr + 0x18c);
	        }
	        else if (val == 0x14)
	        {
        	        // usb hsirq
                	val = readl(dev->base_addr + 0x18c);
	                val &= 0xffffff00;
        	        val |= 0x00000020;              
                	writel(val, dev->base_addr + 0x18c);
        	}
	        else if (val == 0x24)
        	{
	                //EP1 in IRQ
        	        val = readl(dev->base_addr + 0x188);
                	val &= 0xffffff00;
	                val |= 0x00000002;
        	        writel(val, dev->base_addr + 0x188);
	        }

	    }
	    //msleep(1);
	}
	return 0;
}
#endif
// end for kernel thread

static int __devinit
kagen2_plat_probe(struct platform_device *pdev)
{
        struct kagen2 *dev;
        int ret; int i;
        unsigned int irqflags;
        resource_size_t base, len;
        struct resource *iomem, *irq_res;

printk("kagen2_plat_probe 1\n");
        irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!irq_res || !iomem) {
printk("kagen2_plat_probe 2\n");
                return -EINVAL;
        }

        if (!irq_res->start) {
printk("kagen2_plat_probe 3\n");
                return (-ENODEV);
        }

        /* alloc, and start init */
        dev = kzalloc(sizeof(struct kagen2), GFP_KERNEL);
        if (!dev)
	{
printk("kagen2_plat_probe 4\n");
                return (-ENOMEM);
	}
printk("kagen2_plat_probe 5\n");

        spin_lock_init(&dev->lock);
        dev->irq = irq_res->start;
        dev->dev = &(pdev->dev);
        dev->gadget.ops = &kagen2_gadget_ops;
        dev->gadget.max_speed = USB_SPEED_HIGH;

        /* the "gadget" abstracts/virtualizes the controller */
        dev_set_name(&dev->gadget.dev, "gadget");
        dev->gadget.dev.parent = &(pdev->dev);
        dev->gadget.dev.release = kagen2_gadget_release;
        dev->gadget.name = driver_name;

        irqflags = 0;
        if (irq_res->flags & IORESOURCE_IRQ_HIGHEDGE)
                irqflags |= IRQF_TRIGGER_RISING;
        if (irq_res->flags & IORESOURCE_IRQ_LOWEDGE)
                irqflags |= IRQF_TRIGGER_FALLING;
        if (irq_res->flags & IORESOURCE_IRQ_HIGHLEVEL)
                irqflags |= IRQF_TRIGGER_HIGH;
        if (irq_res->flags & IORESOURCE_IRQ_LOWLEVEL)
                irqflags |= IRQF_TRIGGER_LOW;

        base = iomem->start;
        len = resource_size(iomem);

        if (!request_mem_region(base, len, driver_name)) {
                ret = -EBUSY;
                goto err;
        }
        dev->base_addr = ioremap_nocache(base, len);
        if (!dev->base_addr) {
                ret = -EFAULT;
                goto err_req;
        }

printk("kagen2_plat_probe 6, 0x%x 0x%x\n", dev->base_addr, len);
	// init low level usb
	kagen2_lowlevel_init(dev);

        // set usb irq 32 to edge trigger 
        //ka2000_writel(0x1, 0xa000602c);
	// clear irq
	//ka2000_writel(0x0, 0xa0006030);
	// unmask irq 32
        //ka2000_writel((0xfffffffe), 0xa0006014);

	// init usb software structure
        for (i = 0; i < NUM_ENDPOINTS; ++i) {
                struct kagen2_ep *ep = &dev->ep[i];

                ep->ep.name = ep_name[i];
                ep->dev = dev;

		ep->desc = NULL;
        	INIT_LIST_HEAD(&ep->queue);

	        ep->ep.maxpacket = ~0;
        	ep->ep.ops = &kagen2_ep_ops;
        }
        dev->ep[0].ep.maxpacket = 64;
        dev->gadget.ep0 = &dev->ep[0].ep;
        INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);

printk("kagen2_plat_probe 8\n");
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	list_add_tail(&dev->ep[1].ep.ep_list, &dev->gadget.ep_list);
	list_add_tail(&dev->ep[2].ep.ep_list, &dev->gadget.ep_list);
        dev->ep[1].fifo_size = dev->ep[2].fifo_size = 512;
        dev->ep[1].ep.maxpacket = dev->ep[2].ep.maxpacket = 512;

	// unmask the irq 32 which is the usb irq
	//ka2000_writel((ka2000_readl(0xa0006014) & 0xfffffffe), 0xa0006014);
	
printk("register irq %d\n", dev->irq);
	ret = request_irq(dev->irq, kagen2_irq, irqflags, driver_name, dev);
	//ret = request_irq(32, kagen2_irq, IRQF_DISABLED, "test", NULL);
        if (ret) {
printk("register irq error\n");
               goto err;
        }
	else {
		tasklet_init(&tskLet_USB, tskLetISR_USB, 0);
		sema_init(&sem, 0);
		spin_lock_init(&usblock);
	}

        ret = device_register(&dev->gadget.dev);
	ret = usb_add_gadget_udc(dev->dev, &dev->gadget);

        platform_set_drvdata(pdev, dev);

	// kernel thread
	//task = kthread_run(&example_thread, (void *)dev, "example_t");
	task = kthread_run(&chkbusy_thread, (void *)dev, "chkbusy_t");

	// set the host to busy '1' now, the ep1_out() will clear this to zero when EP1_OUT is triggered
	dev->host_busy = 1;
	// init wait queue
	init_waitqueue_head(&short_queue);
        //init_waitqueue_entry(&wait, current);
        //add_wait_queue(&short_queue, &wait);

	return 0;
err_req:
	release_mem_region(base, len);
err:
	return ret;
}

static int __devexit
kagen2_plat_remove(struct platform_device *pdev)
{
        struct kagen2 *dev = platform_get_drvdata(pdev);

	// remove wait queue
	//remove_wait_queue(&short_queue, &wait);

        release_mem_region(pdev->resource[0].start,
                resource_size(&pdev->resource[0]));

        kfree(dev);

        return 0;
}

/*-------------------------------------------------------------------------*/

static struct platform_driver kagen2_plat_driver = {
        .probe =        kagen2_plat_probe,
        .remove =       kagen2_plat_remove,
	.driver = {
	        .name =      driver_name,
        	.owner =     THIS_MODULE,
	},
};

static int __init kagen2_init (void)
{
	int ret;

printk("kagen2_init\n");
        ret = platform_driver_register(&kagen2_plat_driver);
printk("kagen2_init %d\n", ret);

	return ret;
}
module_init (kagen2_init);

static void __exit kagen2_cleanup (void)
{
        platform_driver_unregister(&kagen2_plat_driver);

	free_irq(32, NULL);

	//kthread_stop(task);
}
module_exit (kagen2_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Keyasic");
MODULE_LICENSE("GPL");
module_param(myirq, int, 0);
